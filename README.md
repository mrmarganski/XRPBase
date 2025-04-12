# XRP Robot Base Build and Programming Guide (WPILib Java)

## Overview
This guide provides step-by-step instructions for building and programming an XRP robot base for an FRC (FIRST Robotics Competition) team using WPILib and Java. The XRP (Experimental Robotics Platform), shown below, is a modular platform ideal for educational robotics.

![XRP Robot Overview](https://www.sparkfun.com/pictures/3/6/7/367641-XRP_Promo_01.jpg)  
*Image source: SparkFun Electronics*

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
- Motor controller (e.g., Spark Mini or Talon SRX)
- Microcontroller (e.g., Raspberry Pi Pico or roboRIO-compatible board)
- Battery pack (7.2V NiMH or 11.1V LiPo, with charger)
- Wires (22–24 AWG, assorted colors)
- Zip ties and Velcro straps
- USB cable (for programming)

### Software Requirements
- WPILib Suite (latest version, includes Java support)
- Java Development Kit (JDK 11 or 17, as per WPILib requirements)
- VS Code (with WPILib extension)
- XRP Firmware (latest version, download from [XRP documentation](https://xrp-docs.readthedocs.io/))
- FRC Driver Station (for testing)

---

## Build Instructions

### Step 1: Assemble the Chassis
1. Unpack the XRP Robot Base Kit and verify all components.
2. Attach the base plate to the side panels using provided screws and hex keys.
3. Secure the four motor mounts to the chassis, ensuring proper alignment for wheels.
   - **Tip**: Confirm motors are oriented for differential drive (two left, two right).

![XRP Chassis Assembly](https://cdn.sparkfun.com/r/600-600/assets/learn_tutorials/3/0/9/7/XRP_Assembly_01.jpg)  
*Image source: SparkFun Electronics*

### Step 2: Install Motors and Wheels
1. Mount each DC motor to a motor mount with screws.
2. Attach wheels to motor shafts, ensuring a secure fit (use set screws if included).
3. Check that wheels rotate freely and are parallel to the chassis.

### Step 3: Electrical Setup
1. Mount the motor controller to the chassis (use Velcro or screws).
2. Connect motor wires to the controller:
   - Left motors to ports L1 and L2.
   - Right motors to ports R1 and R2.
3. Secure the microcontroller to the chassis, keeping USB ports accessible.
4. Wire the battery pack to the motor controller’s power input.
   - **Safety Note**: Verify polarity to prevent damage.
5. Use zip ties to organize and secure all wiring.

![XRP Wiring Example](https://cdn.sparkfun.com/r/600-600/assets/learn_tutorials/3/0/9/7/XRP_Wiring.jpg)  
*Image source: SparkFun Electronics*

### Step 4: Test the Hardware
1. Connect the battery and power on the system.
2. Manually test each wheel’s rotation (if controller supports manual mode).
3. Inspect for loose connections or misaligned components.

---

## Programming Instructions

### Step 1: Set Up the Development Environment
1. Install WPILib Suite and JDK on your laptop (follow [WPILib installation guide](https://docs.wpi.first.org/)).
2. Flash the XRP firmware to the microcontroller via USB.
3. Open VS Code and create a new WPILib project:
   - Select “Java” > “Timed Robot” template.
   - Name the project (e.g., `XRPRobotBase`).

### Step 2: Write Basic Drive Code
Below is a sample Java program for a differential drive robot using WPILib’s Java framework.

```java
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.Joystick;

public class Robot extends TimedRobot {
  // Motor controllers
  private PWMSparkMax leftMotor1;
  private PWMSparkMax leftMotor2;
  private PWMSparkMax rightMotor1;
  private PWMSparkMax rightMotor2;

  // Motor groups
  private MotorControllerGroup leftMotors;
  private MotorControllerGroup rightMotors;

  // Differential drive
  private DifferentialDrive drive;

  // Joystick
  private Joystick joystick;

  @Override
  public void robotInit() {
    // Initialize motors (adjust PWM ports as needed)
    leftMotor1 = new PWMSparkMax(1);
    leftMotor2 = new PWMSparkMax(2);
    rightMotor1 = new PWMSparkMax(3);
    rightMotor2 = new PWMSparkMax(4);

    // Invert right motors if needed
    rightMotor1.setInverted(true);
    rightMotor2.setInverted(true);

    // Group motors
    leftMotors = new MotorControllerGroup(leftMotor1, leftMotor2);
    rightMotors = new MotorControllerGroup(rightMotor1, rightMotor2);

    // Initialize differential drive
    drive = new DifferentialDrive(leftMotors, rightMotors);

    // Initialize joystick (port 0)
    joystick = new Joystick(0);
  }

  @Override
  public void teleopPeriodic() {
    // Arcade drive: forward/back from Y-axis, turn from X-axis
    drive.arcadeDrive(
        -joystick.getY() * 0.8, // Scale speed for safety
        joystick.getX() * 0.7   // Scale turn for smoother control
    );
  }
}