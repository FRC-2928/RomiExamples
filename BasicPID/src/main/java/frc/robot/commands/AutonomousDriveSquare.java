// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutonomousDriveSquare extends SequentialCommandGroup {
  /**
   * Creates a new Autonomous Drive based on distance. This will drive out for a specified distance,
   * turn around and drive back.
   *
   * @param drivetrain The drivetrain subsystem on which this command will run
   */
  
  public AutonomousDriveSquare(Drivetrain drivetrain) {
    // Stop the robot
    drivetrain.arcadeDrive(0, 0);

    // Reset the Odometry
    drivetrain.resetGyro();
    drivetrain.resetEncoders();

    // Drive in a square
    addCommands(
        new DriveDistanceProfiled(1.0, drivetrain),
        new TurnToAngleProfiled(90, drivetrain),
        new DriveDistanceProfiled(2.0, drivetrain),
        new TurnToAngleProfiled(180, drivetrain),
        new DriveDistanceProfiled(3.0, drivetrain),
        new TurnToAngleProfiled(270, drivetrain),
        new DriveDistanceProfiled(4.0, drivetrain),
        new TurnToAngleProfiled(0, drivetrain));
  }
}