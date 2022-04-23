// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

import java.time.Instant;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutonomousFollowLine extends SequentialCommandGroup {
  /**
   * Creates a new Autonomous Line Following command. This will drive out for a specified distance,
   * turn around and drive back.
   *
   * @param drivetrain The drivetrain subsystem on which this command will run
   */
  public AutonomousFollowLine(Drivetrain drive, Vision vision) {
    addCommands(
        new InstantCommand(drive::resetOdometry, drive),
        new LineFollowPIDCommand(drive, vision),
        new TurnDegrees(-0.5, 180, drive),
        new LineFollowPIDCommand(drive, vision),
        new TurnDegrees(0.5, 180, drive));
  }
}
