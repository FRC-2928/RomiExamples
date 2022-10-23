// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveDistancePID extends PIDCommand {
  /** Creates a new DriveDistancePID. */
  private static Drivetrain m_drive;
  private static NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private static NetworkTable table = inst.getTable("Shuffleboard/Drivetrain");
  
  public DriveDistancePID(double targetDistance, Drivetrain drivetrain) {
    super(
        // The controller that the command will use
        new PIDController(DrivetrainConstants.kPDriveVel,
                          DrivetrainConstants.kIDriveVel,
                          DrivetrainConstants.kDDriveVel),
        // This should return the measurement
        drivetrain::getAverageDistanceMeters,
        // This should return the setpoint (can also be a constant)
        targetDistance,
        // This uses the output
        output -> {
          // Use the output here
          drivetrain.drive(output, 0);
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
    // Override PID parameters from Shuffleboard
    getController().setSetpoint(table.getEntry("Distance").getDouble(0.0));
    getController().setP(table.getEntry("kP").getDouble(DrivetrainConstants.kPDriveVel));
    getController().setD(table.getEntry("kD").getDouble(DrivetrainConstants.kDDriveVel));
    getController().setI(table.getEntry("kI").getDouble(DrivetrainConstants.kIDriveVel));

    // Publish that the command is in-process
    SmartDashboard.putBoolean("Finished", false);
  }
  
  public void execute() {
    super.execute();
    // Keep track of how far away we are from the setpoint
    SmartDashboard.putNumber("Error", getController().getPositionError());
  }

  // Returns true when we're at the setpoint, so the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    // Publish that the command is done
    SmartDashboard.putBoolean("Finished", getController().atSetpoint());
  }

  public void resetOdometry() {
    m_drive.arcadeDrive(0, 0);
    m_drive.resetGyro();
    m_drive.resetEncoders();
  }
}
