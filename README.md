# STM32 IMU + Barometer Flight Sensor System

A bare-metal embedded firmware project for the **STM32F411RE (Nucleo)** that reads an **MPU6050** inertial measurement unit and a **BMP280** barometric pressure sensor over a shared I²C bus, then fuses and processes the data into meaningful flight-relevant outputs: **pitch/roll attitude** (via a complementary filter) and **barometric altitude**.

Built in C using the STM32 HAL, configured with STM32CubeMX/CubeIDE, with results streamed over UART to a serial terminal.

---

## What It Does

- Brings up two I²C sensors on a single shared bus (SCL/SDA), each at its own address.
- Reads the MPU6050's accelerometer and gyroscope (14-byte burst read from `0x3B`).
- Computes **pitch** and **roll** from the accelerometer using `atan2`, and **angular rates** from the gyroscope.
- Fuses the two with a **complementary filter**  high-pass on the gyro, low-pass on the accelerometer to get a clean, drift-corrected attitude estimate.
- Reads the BMP280's raw pressure and temperature, applies **Bosch's calibration/compensation formulas**, and converts pressure to **altitude** using the barometric formula.
- Streams everything over UART (115200 baud) for live monitoring.

---

## Hardware

| Component | Role | Interface | Address |
|-----------|------|-----------|---------|
| STM32F411RE (Nucleo) | Microcontroller | — | — |
| MPU6050 (GY-521) | Accelerometer + gyroscope | I²C | `0x68` (`0xD0` shifted) |
| BMP280 (GY-BMP280) | Pressure + temperature | I²C | `0x76` (`0xEC` shifted) |

**Bus:** I²C1 on **PB6 (SCL)** / **PB7 (SDA)**, shared by both sensors in parallel.
**Output:** USART2 (PA2/PA3) routed through the onboard ST-Link virtual COM port.
**Clock:** 84 MHz (HSI → PLL).

---

## How It Works

### 1. Sensor bring-up
Each sensor is verified by reading its identity register before use — the MPU6050's `WHO_AM_I` (`0x75`) and the BMP280's chip-ID (`0xD0`)  - and only then woken/configured. The MPU6050 boots in sleep mode, so `0x00` is written to its power-management register (`0x6B`) to wake it; the BMP280 is put into normal measurement mode via its control register (`0xF4`).

### 2. Attitude estimation (MPU6050)
Raw 16-bit signed accelerometer values give a gravity vector, from which pitch and roll are derived geometrically. The gyroscope gives angular velocity (scaled by 131 LSB/°·s⁻¹ at the default ±250°/s range). The complementary filter blends them:

```
angle = 0.96 * (angle + gyro_rate * dt) + 0.04 * accel_angle
```

`dt` is measured each loop with `HAL_GetTick()` so the integration uses the true elapsed time, not an assumed constant.

### 3. Pressure & altitude (BMP280)
The BMP280 outputs *uncompensated* raw values that are meaningless on their own. The 12 factory calibration coefficients are read once at startup (registers `0x88`–`0x9F`, little-endian, mixed signed/unsigned), then fed through Bosch's integer compensation routines to produce real temperature (°C) and pressure (Pa). Pressure is converted to altitude with the standard barometric formula.

---

## Where to Look (Code Tour)

Everything of interest lives in **`Core/Src/main.c`**, inside the `USER CODE` sections:

- **`BMP280_T_Compensation()` / `BMP280_P_Compensation()`** — the Bosch calibration math that turns raw sensor counts into real temperature and pressure.
- **`I2C_BusReset()`** — a manual bus-recovery routine (toggles SCL to free a stuck bus) that runs before I²C initialization. See the "Challenges" section for why this exists.
- **`main()` → `USER CODE BEGIN 2`** — sensor identification, wake-up, and the one-time read of the BMP280 calibration coefficients.
- **`main()` → `USER CODE BEGIN 3`** (the main loop) — the live read/process/print cycle: IMU burst read → complementary filter → BMP280 read → compensation → UART output.
- **`I2C_BusReset` placement** in `main()` — note it runs *between* GPIO init and I²C init; the ordering matters.

The `.ioc` file can be opened in STM32CubeIDE to see the peripheral configuration (I²C1, USART2, clock tree).

---

## Building & Running

1. Open the `.ioc` in **STM32CubeIDE** (or import the project directly).
2. Build (the HAL drivers are included, so it should compile as-is).
3. Flash to a Nucleo-F411RE via the onboard ST-Link.
4. Open a serial terminal on the **ST-Link Virtual COM Port** at **115200 baud**.
5. Press the board's reset button to see the startup identification lines, followed by the live data stream.

Wiring: both sensors share PB6 (SCL) and PB7 (SDA), powered from 3V3 and GND. The MPU6050's `AD0` pin is tied to GND to fix its address at `0x68`. No external pull-up resistors are needed — the breakout boards include their own.

---

## What I Learned

This was my first embedded systems project from scratch, and the value was as much in the debugging as in the final result. A few things I came away understanding:

- **The STM32 toolchain and project structure** — how CubeMX configuration maps to generated HAL code, the role of the `.ioc`, why the toolchain target (STM32CubeIDE vs. IAR/EWARM) matters, and how to keep custom code inside the `USER CODE` guards so it survives regeneration.
- **I²C at a low level** — addressing and the 7-bit-to-8-bit shift, register-based reads/writes (`HAL_I2C_Mem_Read/Write`), the difference between bus-level status (NACK, BUSY, timeout) and device-level identity checks, and how pull-ups and the shared-bus topology actually work.
- **Sensor fusion fundamentals** — why a gyroscope alone drifts, why an accelerometer alone is noisy, and how a complementary filter combines them; the importance of measuring real `dt`.
- **Fixed-point / integer math** — reading the BMP280 datasheet's compensation formulas, handling signed vs. unsigned coefficients correctly, and printing decimals without floating point.
- **Hardware reality** — that the code looking right and the project actually working are two different things, and how a methodical approach with the multimeter can be helpful in these situations.

---

## Debugging Challenges 

The bring-up was not smooth, and working through the failures was the most instructive part:

- **I²C bus stuck `BUSY`:** the peripheral reported `HAL_BUSY` on the very first transaction, before touching any device. Isolated it to the MCU side with a sensor-disconnect test, and resolved it with a manual bus-reset routine plus correcting the system clock (the project had defaulted to a bare 16 MHz HSI with the PLL off, giving I²C an unhealthy peripheral clock).
- **Physical connections:** persistent NACKs traced to marginal breadboard contacts — a sensor measuring 3.3 V at idle (held up by the *other* sensor's pull-ups) while not actually being electrically connected. Found by measuring each sensor's signal pins independently and confirming with an I²C address scanner.
- **Clone chip identity:** the MPU6050 returned a `WHO_AM_I` of `0x70` instead of the expected `0x68` — a common trait of clone modules. The chip is otherwise fully register-compatible; the ID check just needed updating.

---

## Possible Next Steps

- Zero the altitude at launch to report **height above takeoff** rather than absolute MSL.
- Add yaw / full 3-axis orientation.
- Move from a breadboard to a **custom PCB** integrating the MCU and both sensors.
- Log data to onboard storage for post-flight analysis.

---

*Built on an STM32F411RE Nucleo with an MPU6050 and BMP280, in C with the STM32 HAL.*
