// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class RoadFollow extends CommandBase {
  private final Drivetrain m_drive;
  private final Vision m_vision;

  /** Creates a new RoadFollow. */
  public RoadFollow(Drivetrain drive, Vision vision) {
    m_drive = drive;
    m_vision = vision;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.arcadeDrive(0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Double xaxisSpeed = m_vision.getXAxisSpeed();
    Double zaxisRotate = m_vision.getZAxisRotate();
    m_drive.arcadeDrive(xaxisSpeed, zaxisRotate);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
