// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveDistanceProfiled extends ProfiledPIDCommand {

  private static Drivetrain m_drivetrain;

  private static NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private static NetworkTable m_table = inst.getTable("Shuffleboard/Drivetrain");
  
  /** Creates a new DriveDistanceProfiled. */
  public DriveDistanceProfiled(double targetDistance, Drivetrain drivetrain) {
    super(
        // The ProfiledPIDController used by the command
        new ProfiledPIDController(
            // The PID gains
            Constants.kPDriveProfiled,
            Constants.kIDriveProfiled,
            Constants.kDDriveProfiled,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(Constants.kMaxSpeedMetersPerSecond, 
                                             Constants.kMaxAccelMetersPerSecondSquared)),
        // This should return the measurement
        () -> drivetrain.getAverageDistanceMeters(),
        // This should return the goal (can also be a constant)
        () -> new TrapezoidProfile.State(targetDistance,0),
        // Use the calculated velocity at each setpoint
        (output, setpoint) -> {
          // drivetrain.arcadeDrive(output, 0);
          drivetrain.setOutputMetersPerSecond(output, setpoint.velocity, setpoint.position);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);    

    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(0.06, 0.05);

    m_drivetrain = drivetrain;
  }

  public void initialize() { 
    // Reset the Odometry 
    m_drivetrain.resetEncoders();
    m_drivetrain.resetGyro();
    super.initialize();

    // Override PID parameters from Shuffleboard
    getController().setP(m_table.getEntry("DriveProfiledkP").getDouble(Constants.kPDriveProfiled));
    getController().setD(m_table.getEntry("DriveProfiledkI").getDouble(Constants.kIDriveProfiled));
    getController().setD(m_table.getEntry("DriveProfiledkD").getDouble(Constants.kDDriveProfiled));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atGoal();
  }
}
