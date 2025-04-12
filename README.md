# XRP Robot Base Build and Programming Guide

## Overview
This guide provides step-by-step instructions for building and programming an XRP robot base for an FRC team. The XRP (Experimental Robotics Platform) is a versatile, modular platform designed for educational robotics projects. This document covers hardware assembly, electrical setup, and basic programming to get the robot moving.

---

## Prerequisites

### Tools Required
- Phillips screwdriver (size #2)
- Hex key set (metric, 2mm–5mm)
- Wire cutters/strippers
- Soldering iron (optional, for custom wiring)
- Laptop with USB port

### Materials Needed
- XRP Robot Base Kit (includes chassis, motors, wheels, etc.)
- 4x DC motors (with encoders, if available)
- Motor controller (e.g., Spark Mini or equivalent)
- Microcontroller (e.g., Raspberry Pi Pico or Arduino)
- Battery pack (7.2V NiMH or 11.1V LiPo, with charger)
- Wires (22–24 AWG, assorted colors)
- Zip ties and Velcro straps
- USB cable (for programming)

### Software Requirements
- WPILib (for FRC-compatible programming)
- Python 3.8+ (with `wpilib` and `robotpy` libraries)
- VS Code (with WPILib extension)
- XRP Firmware (latest version, download from [XRP documentation](https://xrp-docs.readthedocs.io/))

---

## Build Instructions

### Step 1: Assemble the Chassis
1. Unpack the XRP Robot Base Kit and verify all parts are present.
2. Attach the base plate to the side panels using provided screws and hex keys.
3. Secure the four motor mounts to the chassis, ensuring alignment for wheel placement.
   - **Tip**: Check that motors are oriented correctly for a differential drive (two left, two right).

### Step 2: Install Motors and Wheels
1. Attach each DC motor to a motor mount using screws.
2. Connect the wheels to the motor shafts, ensuring a tight fit (use set screws if provided).
3. Verify that wheels rotate freely and are aligned parallel to the chassis.

### Step 3: Electrical Setup
1. Mount the motor controller to the chassis (use Velcro or screws).
2. Connect motor wires to the controller:
   - Left motors to ports L1 and L2.
   - Right motors to ports R1 and R2.
3. Attach the microcontroller to the chassis, ensuring access to USB ports.
4. Wire the battery pack to the motor controller’s power input.
   - **Safety Note**: Double-check polarity to avoid damage.
5. Secure all wires with zip ties to prevent loose connections.

### Step 4: Test the Hardware
1. Connect the battery and power on the system.
2. Manually spin each wheel to confirm motor response (if controller supports manual mode).
3. Check for loose components or wiring issues.

---

## Programming Instructions

### Step 1: Set Up the Development Environment
1. Install WPILib and Python on your laptop (follow [WPILib installation guide](https://docs.wpilib.org/)).
2. Install the XRP firmware on the microcontroller via USB.
3. Open VS Code and create a new WPILib project:
   - Select “RobotPy” template.
   - Name the project (e.g., `XRP_Robot_Base`).

### Step 2: Write Basic Drive Code
Below is a sample Python program for a differential drive robot using `robotpy`.

```python
import wpilib
import wpilib.drive
import ctre  # For motor controllers, if using Talon/Spark

class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        # Initialize motor controllers
        self.left_motor1 = ctre.WPI_TalonSRX(1)  # Adjust port numbers
        self.left_motor2 = ctre.WPI_TalonSRX(2)
        self.right_motor1 = ctre.WPI_TalonSRX(3)
        self.right_motor2 = ctre.WPI_TalonSRX(4)

        # Group motors for differential drive
        self.left_motors = wpilib.MotorControllerGroup(self.left_motor1, self.left_motor2)
        self.right_motors = wpilib.MotorControllerGroup(self.right_motor1, self.right_motor2)

        # Create differential drive object
        self.drive = wpilib.drive.DifferentialDrive(self.left_motors, self.right_motors)

        # Initialize joystick
        self.joystick = wpilib.Joystick(0)

    def teleopPeriodic(self):
        # Arcade drive: forward/back from Y-axis, turn from X-axis
        self.drive.arcadeDrive(
            -self.joystick.getY() * 0.8,  # Scale speed for safety
            self.joystick.getX() * 0.7   # Scale turn for smoother control
        )

if __name__ == "__main__":
    wpilib.run(MyRobot)