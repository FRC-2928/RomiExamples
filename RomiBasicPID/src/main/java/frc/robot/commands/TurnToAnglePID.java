// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurnToAnglePID extends PIDCommand {

  private static NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private static NetworkTable m_table = inst.getTable("Shuffleboard/Drivetrain");

  /** Creates a new TurnToAnglePID. */
  public TurnToAnglePID(double targetAngleDegrees, Drivetrain drivetrain) {
    super(
        // The controller that the command will use
        new PIDController(Constants.kPTurnVel,
                          Constants.kITurnVel, 
                          Constants.kDTurnVel),
        // This should return the measurement
        () -> drivetrain.getHeading(),
        // This should return the setpoint (can also be a constant)
        () -> targetAngleDegrees,
        // This uses the output
        output -> {
          // Use the output here
          drivetrain.arcadeDrive(0, MathUtil.clamp(-output, -0.5, 5.0));
        });

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);

    // Configure additional PID options by calling `getController` here.
    getController().enableContinuousInput(-180, 180);
    getController().setTolerance(5.0, 10.0);
  }

  public void initialize() {
    super.initialize();
    // Override PID parameters from Shuffleboard
    getController().setP(m_table.getEntry("TurnKP").getDouble(Constants.kPTurnVel));
    getController().setD(m_table.getEntry("TurnkI").getDouble(Constants.kITurnVel));
    getController().setD(m_table.getEntry("TurnkD").getDouble(Constants.kDTurnVel));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
