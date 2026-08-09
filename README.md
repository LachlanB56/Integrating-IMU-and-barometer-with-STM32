# STM32 IMU + Barometer Self-Stabilizing Plane

A bare-metal embedded firmware project for the **STM32F411RE (Nucleo)** that reads an **MPU6050** inertial measurement unit and a **BMP280** barometric pressure sensor over a shared I²C bus, fuses the data into a live pitch/roll attitude estimate and altitude, and closes the loop with two **PID controllers** driving a pair of aileron servos toward wings-level, self-stabilizing flight.

Built in C using the STM32 HAL, configured with STM32CubeMX/CubeIDE, with results streamed over UART to a serial terminal.

---

## What It Does

- Brings up two I²C sensors on a single shared bus (SCL/SDA), each at its own address.
- Reads the MPU6050's accelerometer and gyroscope (14-byte burst read from `0x3B`).
- Computes **pitch** and **roll** from the accelerometer using `atan2`, and **angular rates** from the gyroscope.
- Fuses the two with a **complementary filter** — high-pass on the gyro, low-pass on the accelerometer — to get a clean, drift-corrected attitude estimate.
- Reads the BMP280's raw pressure and temperature, applies **Bosch's calibration/compensation formulas**, and converts pressure to **altitude** using the barometric formula.
- Feeds the filtered pitch/roll into two independent **PID controllers**, whose outputs are mixed and written to two servos (wired as ailerons) to drive the airframe back toward level.
- Streams everything over UART (115200 baud) for live monitoring.

---

## Hardware

| Component | Role | Interface | Address / Pins |
|-----------|------|-----------|-----------------|
| STM32F411RE (Nucleo) | Microcontroller | — | — |
| MPU6050 (GY-521) | Accelerometer + gyroscope | I²C1 | `0x68` (`0xD0` shifted) |
| BMP280 (GY-BMP280) | Pressure + temperature | I²C1 | `0x76` (`0xEC` shifted) |
| Servo 1 (aileron) | PWM output | TIM3 CH1 | PA6 (AF2) |
| Servo 2 (aileron) | PWM output | TIM3 CH2 | PA7 (AF2) |

**Bus:** I²C1 on **PB6 (SCL)** / **PB7 (SDA)**, shared by both sensors in parallel.
**Output:** USART2 (PA2/PA3) routed through the onboard ST-Link virtual COM port, 115200 baud.
**Clock:** HSI (16 MHz) → PLL → **84 MHz** SYSCLK/HCLK (see below for how this affects servo PWM timing).

---

## How It Works

### 1. Sensor bring-up
Each sensor is verified by reading its identity register before use — the MPU6050's `WHO_AM_I` (`0x75`) and the BMP280's chip-ID (`0xD0`) — and only then woken/configured. The MPU6050 boots in sleep mode, so `0x00` is written to its power-management register (`0x6B`) to wake it; the BMP280 is put into normal measurement mode via its control register (`0xF4`).

### 2. Attitude estimation (MPU6050)
Raw 16-bit signed accelerometer values give a gravity vector, from which pitch and roll are derived geometrically. The gyroscope gives angular velocity (scaled by 131 LSB/°·s⁻¹ at the default ±250°/s range). The complementary filter blends them:

```
angle = 0.96 * (angle + gyro_rate * dt) + 0.04 * accel_angle
```

`dt` is measured each loop with `HAL_GetTick()` so the integration uses the true elapsed time, not an assumed constant.

### 3. Pressure & altitude (BMP280)
The BMP280 outputs *uncompensated* raw values that are meaningless on their own. The 12 factory calibration coefficients are read once at startup (registers `0x88`–`0x9F`, little-endian, mixed signed/unsigned), then fed through Bosch's integer compensation routines to produce real temperature (°C) and pressure (Pa). Pressure is converted to altitude with the standard barometric formula.

### 4. PID stabilization (roll & pitch)
Two independent PID controllers drive the aircraft toward wings-level, zero-pitch flight:

```c
static PID_t pid_roll  = {.kp=3.3f, .ki=1.1f, .kd=0.5f, .i_limit=100.0f, .out_limit=350.0f};
static PID_t pid_pitch = {.kp=4.1f, .ki=0.8f, .kd=0.3f, .i_limit=100.0f, .out_limit=350.0f};
```

Each call to `PID_Update()` takes a setpoint of `0.0f` (level), the current filtered pitch/roll, and `dt`, and returns a clamped correction:

- Proportional + integral term computed on error, with the integral clamped (`i_limit`) to prevent windup.
- Derivative term computed on the measurement rather than the error, to avoid derivative kick when the setpoint changes.
- Output clamped to `±out_limit` before being applied to the servos.

The two correction outputs are mixed into differential aileron commands:

```c
float roll_out  = PID_Update(&pid_roll,  0.0f, filteredroll,  dt);
float pitch_out = PID_Update(&pid_pitch, 0.0f, filteredpitch, dt);

Servo_Write(&htim3, TIM_CHANNEL_1, SERVO_MID_US + pitch_out + roll_out);
Servo_Write(&htim3, TIM_CHANNEL_2, SERVO_MID_US + pitch_out - roll_out);
```

Adding and subtracting the roll term between the two channels drives the ailerons in opposite directions for roll correction while moving together for pitch correction (elevon-style mixing). `Servo_Write()` is a thin wrapper around `__HAL_TIM_SET_COMPARE` that clamps the pulse width to `SERVO_MIN_US`/`SERVO_MAX_US` before writing it.

**PID gains are not yet flight-tuned** — the values above are a starting point and will likely need bench/ground testing (or a Ziegler–Nichols-style sweep) once mounted in the airframe.

### 5. Servo PWM timing note
TIM3 is configured with `Prescaler = 99` and `Period = 19999`. At the 84 MHz APB1 timer clock this gives a ~840 kHz counter tick and a PWM frequency of **~42 Hz**, rather than the textbook 50 Hz — one timer count works out to ~1.19 µs, not exactly 1 µs. So the `SERVO_MIN_US`/`SERVO_MID_US`/`SERVO_MAX_US` constants are close to, but not exactly, microseconds — actual pulses land roughly in the 1.19–2.38 ms range. Most analog hobby servos tolerate this fine, but for textbook-accurate 50 Hz/1000–2000 µs timing the prescaler would need retuning to give TIM3 a true 1 MHz tick.

---

## Where to Look (Code Tour)

Everything of interest lives in **`Core/Src/main.c`**, inside the `USER CODE` sections:

- **`BMP280_T_Compensation()` / `BMP280_P_Compensation()`** — the Bosch calibration math that turns raw sensor counts into real temperature and pressure.
- **`I2C_BusReset()`** — a manual bus-recovery routine (toggles SCL to free a stuck bus) that runs before I²C initialization. See the "Debugging Challenges" section for why this exists.
- **`PID_Update()`** — the generic PID step function used by both `pid_roll` and `pid_pitch`.
- **`Servo_Write()`** — clamps and writes a pulse-width value to a TIM3 PWM channel.
- **`main()` → `USER CODE BEGIN 2`** — sensor identification, wake-up, PWM start, and the one-time read of the BMP280 calibration coefficients.
- **`main()` → `USER CODE BEGIN 3`** (the main loop) — the live read/process/print cycle: IMU burst read → complementary filter → BMP280 read → compensation → PID update → servo write → UART output.

The `.ioc` file can be opened in STM32CubeIDE to see the full peripheral configuration (I²C1, TIM3, USART2, clock tree).

---

## Building & Running

1. Open the `.ioc` in **STM32CubeIDE** (or import the project directly).
2. Build (the HAL drivers, including TIM, are included, so it should compile as-is).
3. Flash to a Nucleo-F411RE via the onboard ST-Link.
4. Open a serial terminal on the **ST-Link Virtual COM Port** at **115200 baud**.
5. Press the board's reset button to see the startup identification lines, followed by the live pitch/roll/pressure/altitude data stream.

Wiring: both sensors share PB6 (SCL) and PB7 (SDA), powered from 3V3 and GND. The MPU6050's `AD0` pin is tied to GND to fix its address at `0x68`. No external pull-up resistors are needed — the breakout boards include their own. Servo signal wires connect to PA6 and PA7, with servo power/ground run separately (not from the Nucleo's 3V3 rail).

---

## What I Learned

This was my first embedded systems project from scratch, and the value was as much in the debugging as in the final result. A few things I came away understanding:

- **The STM32 toolchain and project structure** — how CubeMX configuration maps to generated HAL code, the role of the `.ioc`, why the toolchain target (STM32CubeIDE vs. IAR/EWARM) matters, and how to keep custom code inside the `USER CODE` guards so it survives regeneration.
- **I²C at a low level** — addressing and the 7-bit-to-8-bit shift, register-based reads/writes (`HAL_I2C_Mem_Read/Write`), the difference between bus-level status (NACK, BUSY, timeout) and device-level identity checks, and how pull-ups and the shared-bus topology actually work.
- **Sensor fusion fundamentals** — why a gyroscope alone drifts, why an accelerometer alone is noisy, and how a complementary filter combines them; the importance of measuring real `dt`.
- **Fixed-point / integer math** — reading the BMP280 datasheet's compensation formulas, handling signed vs. unsigned coefficients correctly, and printing decimals without floating point.
- **Timer/PWM configuration** — how prescaler and period translate a timer's input clock into a PWM frequency and pulse width, and that getting "close enough" for a hobby servo is not the same as hitting the textbook 50 Hz/1–2 ms spec.
- **Hardware reality** — that the code looking right and the project actually working are two different things, and how a methodical approach with the multimeter can be helpful in these situations.

---

## Debugging Challenges

The bring-up was not smooth, and working through the failures was the most instructive part:

- **I²C bus stuck `BUSY`:** the peripheral reported `HAL_BUSY` on the very first transaction, before touching any device. Isolated it to the MCU side with a sensor-disconnect test, and resolved it with a manual bus-reset routine plus correcting the system clock (the project had defaulted to a bare 16 MHz HSI with the PLL off, giving I²C an unhealthy peripheral clock).
- **Physical connections:** persistent NACKs traced to marginal breadboard contacts — a sensor measuring 3.3 V at idle (held up by the *other* sensor's pull-ups) while not actually being electrically connected. Found by measuring each sensor's signal pins independently and confirming with an I²C address scanner.
- **Clone chip identity:** the MPU6050 returned a `WHO_AM_I` of `0x70` instead of the expected `0x68` — a common trait of clone modules. The chip is otherwise fully register-compatible; the ID check just needed updating.

---

## Known Limitations / TODO

- TIM3's prescaler gives a ~42 Hz PWM refresh rather than the standard 50 Hz — works with most analog servos but isn't textbook-accurate; see the "Servo PWM timing note" above.
- PID gains are untuned defaults and need bench/flight tuning.
- No gyro bias calibration step on boot (MPU6050 is only woken from sleep, not zero-offset calibrated).
- `I2C_BusReset()` is implemented but not currently called from `main()`.
