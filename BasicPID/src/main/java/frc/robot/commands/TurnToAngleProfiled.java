// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;

/** A command that will turn the robot to the specified angle using a motion profile. */
public class TurnToAngleProfiled extends ProfiledPIDCommand {
  /**
   * Turns to robot to the specified angle using a motion profile.
   *
   * @param targetAngleDegrees The angle to turn to
   * @param drive The drive subsystem to use
   */
  public TurnToAngleProfiled(double targetAngleDegrees, Drivetrain drivetrain) {
    super(
        new ProfiledPIDController(
            DrivetrainConstants.kPTurnVelProfiled,
            DrivetrainConstants.kITurnVelProfiled,
            DrivetrainConstants.kDTurnVelProfiled,
            DrivetrainConstants.kTrapezoidProfileTurnConstraints),
        // Close loop on heading
        drivetrain::getHeading,
        // Set reference to target
        targetAngleDegrees,
        // Pipe output to turn robot
        (output, setpoint) -> drivetrain.turn(-output));

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);

    // Set the controller to be continuous (because it is an angle controller)
    // getController().enableContinuousInput(-180, 180);
    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    getController()
        .setTolerance(DrivetrainConstants.kTurnToleranceDeg, DrivetrainConstants.kTurnRateToleranceDegPerS);
  }

  public void execute() {
    super.execute(); 
    SmartDashboard.putNumber("(deg.) setpoint", getController().getSetpoint().position);
    SmartDashboard.putNumber("(deg.) Pos. Error", getController().getPositionError());
    SmartDashboard.putBoolean("(deg.) atGoal", getController().atGoal());
  }
  
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("(deg.) atGoal", getController().atGoal());
  }

  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return getController().atGoal();
  }
}
