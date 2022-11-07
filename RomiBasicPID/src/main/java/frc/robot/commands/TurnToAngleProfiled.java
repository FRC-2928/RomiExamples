// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurnToAngleProfiled extends ProfiledPIDCommand {
  /** Creates a new TurnToAngleProfiled. */
  public TurnToAngleProfiled(double targetAngleDegrees, Drivetrain drivetrain) {
    super(
        // The ProfiledPIDController used by the command
        new ProfiledPIDController(
            // The PID gains
            Constants.kPTurnProfiled,
            Constants.kITurnProfiled,
            Constants.kDTurnProfiled,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(Constants.kMaxTurnRateDegPerS, 
                                             Constants.kMaxTurnAccelDegPerSSquared)),

        // This should return the measurement
        () -> drivetrain.getHeading(),

        // This should return the goal (can also be a constant)
        () -> new TrapezoidProfile.State(targetAngleDegrees,0),

        // This uses the output
        (output, setpoint) -> {
          // Use the output (and setpoint, if desired) here
          drivetrain.arcadeDrive(0, MathUtil.clamp(-output, -0.5, 5.0));
        });
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);

    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(5.0, 10.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atGoal();
  }
}
