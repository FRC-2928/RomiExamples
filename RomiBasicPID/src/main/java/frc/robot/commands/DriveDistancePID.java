// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveDistancePID extends PIDCommand {

  private static NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private static NetworkTable m_table = inst.getTable("Shuffleboard/Drivetrain");
  
  /** Creates a new DriveDistancePID. */
  public DriveDistancePID(double targetDistance, Drivetrain drivetrain) {
    super(
        // The controller that the command will use
        new PIDController(Constants.kPDriveVel,
                          Constants.kIDriveVel,
                          Constants.kDDriveVel),
        // This should return the measurement
        () -> drivetrain.getAverageDistanceMeters(),
        // drivetrain::getAverageDistanceMeters,
        // This should return the setpoint (can also be a constant)
        () -> targetDistance,
        // This uses the output
        output -> {
          // Use the output here
          drivetrain.arcadeDrive(output, 0);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);

    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(0.05, 0.05);
  }

  public void initialize() {
    super.initialize();
    // Override PID parameters from Shuffleboard
    getController().setSetpoint(m_table.getEntry("Distance").getDouble(0.0));
    getController().setP(m_table.getEntry("DistancekP").getDouble(Constants.kPDriveVel));
    getController().setI(m_table.getEntry("DistancekI").getDouble(Constants.kIDriveVel));
    getController().setD(m_table.getEntry("DistancekD").getDouble(Constants.kDDriveVel));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
