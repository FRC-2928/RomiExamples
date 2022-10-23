# Basic PID Control
This project is used with the [Motion Control PID](https://2928-frc-programmer-training.readthedocs.io/en/latest/Romi/Control/romiPID/) section of the *Team 2928 FRC Training Guide* documentation in order to demonstrate various programming concepts.  The project is based off of the *RomiReference* example code that's shipped with the WPI Libraries. 

In this program we'll create two new commands that we'll test from the *AutonomousDistance* group command.

- *DriveDistancePID* that will drive the robot in a straight line.

- *TurnToAnglePID* that will allow the robot to turn to a specified angle.  

After that, we'll create two commands to move the robot more smoothly to the desired setpoints, and is an example of a methodology called **Motion Profiling**.

- *DriveDistanceProfiled* that will use a Trapezoid Profile trajectory to drive the robot in a straight line.

- *TurnToAngleProfiled* that will allow the robot to turn to a specified angle using a Trapezoid Profile trajectory.