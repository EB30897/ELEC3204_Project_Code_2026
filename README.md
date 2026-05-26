# ELEC3204_Project_Code_2026

# Advanced Microcontroller Conveyor Control System

An industrial-style conveyor belt drive system simulation designed to demonstrate robust electromechanical automation, power electronics interfaces, and real-time process control. Powered by an **Arduino Uno** and an **L298N Dual H-Bridge Motor Driver**, this firmware relies on a deterministic, non-blocking finite state machine to manage distinct manual and automated operational profiles.

## 🛠️ System Architecture & Subsystems

* **Control Stage (20 Hz Loop):** A deterministic 50ms execution window allocated strictly to high-precision speed measurements, dynamic acceleration/deceleration trajectory tracking, and Proportional-Integral (PI) feedback regulation.
* **Acoustic Proximity Stage (10 Hz Loop):** A 100ms task window processing HC-SR04 ultrasonic telemetry. Operating asynchronously at a slower rate allows physical acoustic echoes to fully dissipate, mitigating signal corruption.
* **Thread-Safe Velocity Estimation:** Leverages External Interrupt channel 0 (`INT0`) on pin `D3` to count quadrature encoder pulses asynchronously. To prevent data corruption on the 8-bit AVR architecture during multi-clock cycle variables reads, global interrupts are momentarily masked during calculations.
* **Hardware Safety Interlocks:** Incorporates a directional lockout mechanism that structurally prevents H-bridge polarity reversals unless the Finite State Machine (FSM) explicitly confirms the conveyor is in a completely static `STOPPED` or `ESTOPPED` state, eliminating destructive back-EMF surges.

## ⚙️ Pin Mapping

| Peripheral | Component Pin | Arduino Uno Pin | Signal Type | Role |
| :--- | :--- | :--- | :--- | :--- |
| **Power Stage** | L298N ENA | **D9** | Output (PWM) | Motor Velocity Regulation |
| | L298N IN1 / IN2 | **D7 / D8** | Output (Digital) | H-Bridge Directional Polarity |
| **User I/O** | START Button | **D4** | Input (Pull-up) | Active-LOW Manual Run Trigger |
| | SENSOR Button | **D5** | Input (Pull-up) | Active-LOW Simulated Sensor Stop |
| **Feedback** | Encoder Output A | **D3** | Input (Interrupt) | Asynchronous Pulse Tracking |
| | Ultrasonic Trig / Echo | **D10 / D11** | I/O (Digital) | Time-of-Flight Proximity Telemetry |

## 🚀 Key Firmware Features
* **Stiction Mitigation:** Applies a dedicated 15ms high-duty "jumpstart" impulse to successfully break static friction before engaging control algorithms.
* **Integral Anti-Windup Protection:** Clamps the internal PI controller accumulator to bounds of $\pm1000.0$ to safeguard against catastrophic runaway acceleration following mechanical conveyor jams.
* **Non-Blocking Timeouts:** Restricts the ultrasonic `pulseIn()` execution block to a strict 25ms cap, ensuring the core motor control routine never starves for CPU cycles.
