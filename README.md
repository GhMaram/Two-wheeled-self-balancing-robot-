# Two-wheeled-self-balancing-robot-(Inverted Pendulum)
This project implements a **two-wheeled self-balancing robot**, a classic and challenging control problem also known as the **inverted pendulum**. The robot is capable of autonomously maintaining its upright position by continuously sensing its orientation and applying corrective motor actions in real time.
The project is a complete hands-on integration of **control theory**, **embedded systems**, **signal processing**, and **real-time motor control**, developed and validated on real hardware.

---

## Project Overview
A self-balancing robot is inherently unstable: without active control, it will immediately fall. The STM32 microcontroller continuously reads orientation data from an IMU, estimates the robot’s tilt angle, computes the control error, and applies a **PID control law** to generate motor commands that keep the robot balanced.
The system operates in real time and reacts to disturbances such as pushes or uneven surfaces.

---

## Control Principle (Inverted Pendulum)
The control loop follows the classical feedback structure :
* **Reference**: Desired pitch angle (upright position, typically 0°)
* **Sensor**: IMU (MPU6050) measuring angular velocity and acceleration
* **Controller**: PID controller computing corrective action
* **Actuator**: DC motors driven via PWM
* **Feedback**: Measured pitch angle fed back to the controller
This closed-loop system continuously minimizes the pitch error to maintain stability.

---

## Control Algorithm

### PID Controller
A **PID (Proportional–Integral–Derivative)** controller is implemented and tuned experimentally:
* **Proportional (P)**: Reacts to the current tilt error
* **Integral (I)**: Eliminates steady-state error
* **Derivative (D)**: Anticipates future error using angular velocity
The controller output directly sets the motor PWM duty cycle, allowing fast and smooth corrective motion.

### Sensor Fusion
Raw IMU signals are noisy and drift over time. To address this, a **complementary filter** is implemented:
* Accelerometer → reliable at low frequency (absolute angle)
* Gyroscope → reliable at high frequency (angular rate)
Combining both provides a stable and responsive pitch estimation suitable for real-time control.

---

## Hardware Components
* STM32F407 Discovery board (ARM Cortex-M4)
* MPU6050 IMU (gyroscope + accelerometer)
* L298N dual H-bridge motor driver
* DC motors with wheels
* Two-wheeled robot chassis
* Battery power supply

---

## Software Architecture

* **Microcontroller**: STM32F407
* **Programming Language**: C
* **IDE**: Keil μ vision
* **Peripherals Used**:
  * I2C for MPU6050 communication
  * Timers for PWM motor control
  * External interrupts for user interaction
  * UART for debugging and real-time monitoring
The firmware is designed around a fast control loop to guarantee stability and responsiveness.

---

## Simulation & Modeling
Before hardware implementation, the system was modeled and validated in **MATLAB/Simulink** :
* Mechanical model of the inverted pendulum
* PID controller block
* Closed-loop stability analysis
Simulation helped guide controller tuning and reduced trial-and-error on hardware.

---

## Real-Time Monitoring
A Bluetooth / serial terminal interface is used to:
* Observe pitch angle, error, and PID output in real time
* Verify controller behavior during calibration and operation
* Assist in PID tuning and debugging
Terminal logs included in the repository show the robot’s dynamic response during operation.

---

## Learning Outcomes
This project strengthened skills in:
* Embedded real-time programming on STM32
* Control systems and PID tuning
* IMU sensor interfacing and sensor fusion
* PWM-based motor control
* Modeling and simulation with Simulink
* Debugging unstable dynamic systems

---

## Result
The robot successfully maintains its balance and recovers from external disturbances, validating both the control design and embedded implementation. Seeing the system stabilize itself in real hardware was the ultimate confirmation of the theory-meets-practice approach.
