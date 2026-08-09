# Integrating IMU and Barometer with STM32 — Self-Stabilizing Plane

STM32CubeIDE project for an STM32F411RE ("Nucleo-F411RE"-class) board that fuses an MPU6050 IMU and a BMP280 barometer over I2C, and uses the resulting attitude estimate to drive two servos (acting as ailerons) through a PID control loop, aiming for a self-leveling / self-stabilizing wing.

## Hardware

| Peripheral | Interface | Pins | Notes |
|---|---|---|---|
| MPU6050 (accel + gyro) | I2C1 | PB6 (SCL), PB7 (SDA) | 7-bit address `0x68` (shifted to `0xD0`/`0xD1` for HAL) |
| BMP280 (pressure + temp) | I2C1 (shared bus) | PB6 (SCL), PB7 (SDA) | 7-bit address `0x76` (shifted to `0xEC`/`0xED`) |
| Servo 1 (aileron) | TIM3 CH1 (PWM) | PA6 | Alternate function AF2 |
| Servo 2 (aileron) | TIM3 CH2 (PWM) | PA7 | Alternate function AF2 |
| Debug/telemetry | USART2 | PA2 (TX), PA3 (RX) | 115200 baud, `printf` retargeted over UART |

## Clock configuration

`SystemClock_Config()` runs the F411 from its internal 16 MHz HSI oscillator through the main PLL:

- HSI (16 MHz) → `/PLLM=8` → 2 MHz PLL input
- `×PLLN=84` → 168 MHz VCO
- `/PLLP=2` → **84 MHz SYSCLK/HCLK**
- APB1 prescaler `/2` → 42 MHz PCLK1 (84 MHz TIM3 clock, since APB1 prescaler ≠ 1 doubles the timer clock)
- APB2 prescaler `/1` → 84 MHz PCLK2
- Flash latency: 2 wait states, voltage scale 1

TIM3 (driving the servos) is configured with `Prescaler = 99` and `Period = 19999`, giving a ~840 kHz counter tick and a **PWM frequency of ~42 Hz** (840 kHz / 20000 counts). Note that at this tick rate, one timer count is ~1.19 µs rather than exactly 1 µs, so the `SERVO_MIN_US`/`SERVO_MID_US`/`SERVO_MAX_US` constants used in code are close to, but not an exact, microsecond scale — actual pulses land roughly in the 1.19–2.38 ms range. Most analog hobby servos tolerate both the slightly off refresh rate and pulse width, but for textbook 50 Hz / 1000–2000 µs servo timing the prescaler would need retuning to give TIM3 a true 1 MHz tick.

## Sensor fusion

On boot, the firmware scans the I2C bus, verifies both sensors respond (MPU6050 `WHO_AM_I`, BMP280 chip ID), wakes the MPU6050 out of sleep, and reads the BMP280's factory calibration coefficients (`dig_T1..3`, `dig_P1..9`) used by the datasheet compensation formulas.

Each control loop iteration:

1. Reads 14 bytes starting at MPU6050 register `0x3B` (accel X/Y/Z, temp, gyro X/Y/Z) over I2C.
2. Computes raw pitch/roll from the accelerometer via `atan2f`.
3. Converts gyro readings to °/s (raw / 131, per the MPU6050 datasheet) and integrates them over `dt`.
4. Combines both with a complementary filter (96% gyro / 4% accelerometer) into `filteredpitch` / `filteredroll`, which suppresses gyro drift while filtering out accelerometer vibration noise.
5. Reads BMP280 raw pressure/temperature, runs the datasheet compensation formulas (`BMP280_T_Compensation` / `BMP280_P_Compensation`), and derives altitude from the barometric formula.

## PID stabilization

Two independent PID controllers (`pid_roll`, `pid_pitch`) drive the aircraft toward wings-level, zero-pitch flight:

```c
static PID_t pid_roll  = {.kp=3.3f, .ki=1.1f, .kd=0.5f, .i_limit=100.0f, .out_limit=350.0f};
static PID_t pid_pitch = {.kp=4.1f, .ki=0.8f, .kd=0.3f, .i_limit=100.0f, .out_limit=350.0f};
```

Each `PID_Update()` call takes a setpoint of `0.0f` (level), the current filtered pitch/roll, and `dt`, and returns a clamped correction:

- Proportional + integral term computed on error, with the integral clamped (`i_limit`) to prevent windup.
- Derivative term computed on the measurement rather than the error, to avoid derivative kick when the setpoint changes.
- Output clamped to `±out_limit` before being applied to the servos.

The two correction outputs are mixed into differential aileron commands and written to the servos via `Servo_Write()` (a thin wrapper around `__HAL_TIM_SET_COMPARE` with pulse-width clamping to `SERVO_MIN_US`/`SERVO_MAX_US`):

```c
float roll_out  = PID_Update(&pid_roll,  0.0f, filteredroll,  dt);
float pitch_out = PID_Update(&pid_pitch, 0.0f, filteredpitch, dt);

Servo_Write(&htim3, TIM_CHANNEL_1, SERVO_MID_US + pitch_out + roll_out);
Servo_Write(&htim3, TIM_CHANNEL_2, SERVO_MID_US + pitch_out - roll_out);
```

Adding and subtracting the roll term between the two channels drives the ailerons in opposite directions for roll correction while moving together for pitch correction (elevon-style mixing).

**PID gains are not yet flight-tuned** — the values above are a starting point and will likely need adjustment (e.g. via ground testing/bench tuning, or a Ziegler–Nichols-style sweep) once mounted in the airframe.

## Telemetry

Every loop iteration prints over USART2 (115200 baud):

```
PITCH = <filtered pitch, deg>
ROLL = <filtered roll, deg>
Temp = <BMP280 temp, C>  Press = <BMP280 pressure, hPa>
Altitude = <derived altitude, m>
```

## Building

Open the project in STM32CubeIDE (`IMUbarometer.ioc` / `.project` / `.cproject`) and build/flash normally, or use the CubeIDE headless build tools. The `Debug/` build output directory is not tracked in version control.

## Known limitations / TODO

- TIM3's prescaler gives a ~42 Hz PWM refresh rather than the standard 50 Hz — works with most analog servos but isn't textbook-accurate; retune the prescaler for a true 1 MHz counter tick if exact 50 Hz/1–2 ms timing is required.
- PID gains are untuned defaults and need bench/flight tuning.
- No gyro bias calibration step on boot (MPU6050 is only woken from sleep, not zero-offset calibrated).
- I2C bus recovery (`I2C_BusReset`) is implemented but not currently called from `main()`.
