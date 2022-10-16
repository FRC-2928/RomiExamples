# Romi Drivetrain Base
This project is used with the sections of the *Team 2928 FRC Training Guide* documentation in order to demonstrate various programming concepts.  The project is based off of the *RomiReference* example code that's shipped with the WPI Libraries. This project serves the following sections of the training guide.

### Basic Robot Structure
For the [Basic Robot Structure](https://2928-frc-programmer-training.readthedocs.io/en/latest/Romi/SC/romiStructure/) section of the training guide the purpose is to explain how an FRC robot program is structured, and to introduce the Drivetrain class.   Changes made to this project are:

- Moved values `kCountsPerRevolution` and `kWheelDiameterInch` into the Constants file.
- Switched distance values from inches to meters.
- Renamed the `m_controller` variable to `m_joystick`.

### Subsystems
In the [Subsystems](https://2928-frc-programmer-training.readthedocs.io/en/latest/Romi/SC/romiSubsystems/) section of the training guide explains what *Subsystems* are and how they are implemented within the robot program. There are two changes made:

- Update the RomiGyro to make use of an Interface.
- Create a method to get the current heading of the robot.

### Commands
The [Commands](https://2928-frc-programmer-training.readthedocs.io/en/latest/Romi/SC/romiCommands/) section of the training guide the explains the use of *Commands* and how they interact with the robot's subsystems.

### Telemetry
The [Telemetry](https://2928-frc-programmer-training.readthedocs.io/en/latest/Romi/SC/romiTelemetry/) section of the training guide the explains telemetry is implemented in the code.