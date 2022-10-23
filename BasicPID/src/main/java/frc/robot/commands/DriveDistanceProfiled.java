// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveDistanceProfiled extends ProfiledPIDCommand {
  /** Creates a new DriveDistanceProfiled. */
  private static Drivetrain m_drive;
  private static NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private static NetworkTable table = inst.getTable("Shuffleboard/Drivetrain");
  
  public DriveDistanceProfiled(double targetDistance, Drivetrain drivetrain) {
    super(
        // The ProfiledPIDController used by the command
        new ProfiledPIDController(
            // The PID gains and motion profile constraints
            DrivetrainConstants.kPDriveVel,
            DrivetrainConstants.kIDriveVel,
            DrivetrainConstants.kDDriveVel,
            // The motion profile constraints
            DrivetrainConstants.kTrapezoidProfileConstraints),

        // The measurement coming from the sensors
        drivetrain::getAverageDistanceMeters,

        // The goal (can also be a constant)
        () -> new TrapezoidProfile.State(targetDistance,0),

        // Use the calculated velocity at each setpoint
        (output, setpoint) -> {
          drivetrain.setOutputMetersPerSecond(output);
        });

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);    

    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(DrivetrainConstants.kDistanceToleranceMeters,
                                 DrivetrainConstants.kVelocityToleranceMetersPerS);

    m_drive = drivetrain;
  }

  public void initialize() {
    super.initialize();

    // Reset the Odometry
    // m_drive.resetEncoders();

    // Override PID parameters from Shuffleboard
    getController().setP(table.getEntry("kP").getDouble(DrivetrainConstants.kPDriveVel));
    getController().setD(table.getEntry("kD").getDouble(DrivetrainConstants.kDDriveVel));
  }

  public void execute() {
    super.execute();
    SmartDashboard.putNumber("goal", getController().getGoal().position);
    SmartDashboard.putNumber("setpoint", getController().getSetpoint().position);
    SmartDashboard.putNumber("Pos. Error", getController().getPositionError());
    SmartDashboard.putNumber("Vel. Error", getController().getVelocityError());
    SmartDashboard.putBoolean("atSetpoint", getController().atSetpoint());
    SmartDashboard.putBoolean("atGoal", getController().atGoal());
  }
  
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("atGoal", getController().atGoal());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atGoal();
  }
}
