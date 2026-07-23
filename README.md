# STM32 Closed-Loop Stepper Motor Controller with PID

This repository contains the firmware for a highly robust, closed-loop stepper motor control system designed around the STM32 microcontroller ecosystem. By integrating an AS5600 magnetic encoder for real-time, absolute positional feedback and employing a precisely tuned Proportional-Integral-Derivative (PID) controller, this architecture ensures zero-loss motor positioning, effectively eliminating the missed-step vulnerability inherent to open-loop stepper configurations.

---

## Table of Contents
1. [System Overview & Architecture](#system-overview--architecture)
2. [Hardware Integration](#hardware-integration)
3. [Deep Dive: Core Components](#deep-dive-core-components)
    - [AS5600 Magnetic Encoder Driver](#1-as5600-magnetic-encoder-driver)
    - [Stepper Motor Control & PWM Generation](#2-stepper-motor-control--pwm-generation)
    - [USB CDC Communication](#3-usb-cdc-communication)
4. [PID Control Loop Mechanics](#pid-control-loop-mechanics)
5. [Fault Detection & Auto-Recovery](#fault-detection--auto-recovery)
6. [Getting Started](#getting-started)

---

## System Overview & Architecture

Standard stepper motors are driven "blind" in an open-loop configuration. While cost-effective, they are prone to losing steps when physical resistance exceeds the motor's holding torque. This project transforms a standard stepper motor into a closed-loop servo mechanism.

**Data Flow Architecture:**
1. **Target Angle Input:** A desired angle (0°–360°) is transmitted via USB Virtual COM Port.
2. **Positional Feedback:** The AS5600 magnetic sensor continuously samples the actual shaft position via I2C.
3. **Error Normalization:** The system calculates the shortest path between the target and measured position, mapped strictly between `[-180.0°, +180.0°]`.
4. **PID Processing:** The normalized error is fed into the PID equation to compute the required corrective velocity.
5. **Actuation:** The output velocity dynamically manipulates the STM32 hardware timer (`TIM2`) Auto-Reload Register (ARR), scaling the PWM frequency fed into a driver (e.g., TB6600) to actuate the motor smoothly.

> **Hardware Note:** Adjust the `max_step_per_second` parameter to roughly 0.70× the theoretical max step rate configured on your external motor driver. Ensure the timer prescaler is set `> 2` to prevent ARR integer clamping at high frequencies.

---

## Hardware Integration

The system leverages multiple hardware peripherals on the STM32 to offload processing overhead from the main CPU.

| STM32 Peripheral | Associated Hardware | Purpose |
| :--- | :--- | :--- |
| **I2C1** | AS5600 Magnetic Encoder | Reads absolute shaft angle via raw memory registers. |
| **TIM2** | External Motor Driver (Step) | Generates high-frequency PWM pulses to drive the stepper coils. |
| **GPIO** | External Motor Driver (Dir) | Controls the rotational vector (Clockwise / Counter-Clockwise). |
| **USB_OTG_FS** | Host PC | Facilitates real-time command reception and telemetry logging via USB CDC. |

---

## Deep Dive: Core Components

### 1. AS5600 Magnetic Encoder Driver
The AS5600 is a contactless, 12-bit high-resolution rotary position sensor.
- **I2C Memory Read:** The firmware polls two 8-bit registers (`AS5600_RAW_ANGLE_H` and `AS5600_RAW_ANGLE_L`).
- **Data Reassembly:** The high and low bytes are concatenated and strictly masked to 12 bits (`raw &= 0x0FFF`).
- **Floating Point Conversion:** The 0–4095 raw integer is multiplied by `0.087890625f` ($360 / 4096$) to yield a precise floating-point degree representation.

### 2. Stepper Motor Control & PWM Generation
The core actuation relies on real-time hardware timer manipulation rather than blocking CPU delays.
- **Frequency Calculation:** The required frequency (steps per second) dictates the timer's ARR limit. The formula used is:
  `ARR = (timer_clock / ((PSC + 1) * frequency)) - 1`
- **Duty Cycle Maintenance:** To guarantee the external stepper driver correctly registers the logic high/low transitions, the Capture/Compare Register (CCR) is perpetually constrained to `ARR / 2`, yielding a mathematically perfect 50% duty cycle.

### 3. USB CDC Communication
Target angles are fed into the system through a non-blocking USB CDC protocol.
- Command payloads are parsed dynamically using standard string delimiters (`\r`, `\n`, or spaces).
- Upon successful parsing, the `stepper_state.target_angle_deg` is overridden, instantly invoking a closed-loop correction response.

---

## PID Control Loop Mechanics

The control loop is executed via the `Stepper_Update()` function, which evaluates on a strict time delta (`dt`), typically a 7 ms interval (~142 Hz).

**Control Sequence:**
1. **Shortest Path Calculation:** Calculates angular error and applies a normalization function (`normalize_angle_error()`) to force the error into a ±180° bound, preventing inefficient full rotations.
2. **Tolerance Deadband:** If `fabsf(error) <= angle_tolerance_deg`, the PWM is safely halted, and integral parameters are zeroed out to prevent micro-oscillations at the target destination.
3. **PID Equation:**
   - **Proportional (P):** Reacts instantly to position divergence. (Recommended: 2.0 to 3.0).
   - **Integral (I):** Resolves lingering steady-state error. Clamped strictly by `i_max` to prevent integral windup. (Recommended: $10^{-3}$).
   - **Derivative (D):** Analyzes the velocity of the error, injecting artificial friction to dampen physical overshoot. (Recommended: $10^{-1}$).
4. **Velocity Conversion:** The abstract PID output is mapped mathematically back to physical motor steps (`steps_per_sec`), bounded by an absolute maximum threshold and a rigid minimum (50 steps/sec) to prevent timer overflow divisions by zero.
5. **Directional Latching:** A critical 20 µs hardware delay is injected between stopping the PWM, swapping the GPIO direction pin, and restarting the PWM. This prevents current shoot-through and missed logic states on older optical-isolated stepper drivers.

---

## Fault Detection & Auto-Recovery

Industrial reliability demands resilience against hardware disconnects or EMI (Electromagnetic Interference) disrupting the I2C bus.

- **Real-Time I2C Validation:** Every sensor poll validates the `HAL_OK` status flag.
- **Automatic Bus Recovery:** Upon consecutive failures, the MCU invokes an I2C software reset, re-initializes the AS5600 data lines, and flushes the I2C registers to break communication deadlocks.
- **Hard Fault Halting:** If the recovery mechanism fails, the system immediately disables the PWM output (preventing dangerous uncontrolled movement) and asserts a fault indicator LED (`GPIOB PIN 2`). 

*(Note: The system also manages a 180° magnetic offset for the primary axis to align physical zero with mathematical zero upon boot).*

---

## Getting Started

1. **Clone the Repository:**
   ```bash
   git clone https://github.com/Adi-pixel26/stm32_stepper_closedloop-pid-.git
   cd stm32_stepper_closedloop-pid-
   ```
2. **STM32CubeIDE Integration:**
   - Launch STM32CubeIDE.
   - Go to `File` ➔ `Import...` ➔ `General` ➔ `Existing Projects into Workspace`.
   - Browse to the project root and click **Finish**.
3. **Compile & Deploy:**
   - Ensure your ST-Link V2 is connected to the Blackpill (`3.3V`, `GND`, `SWDIO`, `SWCLK`).
   - Click the Hammer icon to build, then the Debug icon to flash the firmware.
   - Open a Serial Terminal at `115200` baud to interface with the motor.
