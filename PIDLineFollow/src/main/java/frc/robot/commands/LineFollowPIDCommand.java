/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Drivetrain;

import frc.robot.subsystems.Vision;

public class LineFollowPIDCommand extends PIDCommand {
    private static Drivetrain m_drive;
    private final Vision m_vision;

    public LineFollowPIDCommand(Drivetrain drive, Vision vision) {
        super(
            new PIDController(VisionConstants.kP,0,0), 
            vision::getCenterX, 
            VisionConstants.SETPOINT,
            output -> {
                // Use the output here
                drive.driveLine(-output);
              },
            drive
        );
        m_drive = drive;
        m_vision = vision;
    }

    public void initialize() {
        super.initialize();
        // Make sure everything starts from zero
        m_drive.arcadeDrive(0, 0);
        m_vision.resetFilter();
    }

    public void execute() {
        super.execute();
        SmartDashboard.putNumber("Error", getController().getPositionError());
        SmartDashboard.putBoolean("Finished", false);
    }

    @Override
    public void end(boolean interrupted) {
        // m_drive.arcadeDrive(0, 0);
        SmartDashboard.putBoolean("Finished", true);
    }

    @Override
    public boolean isFinished() {
        return m_vision.getRectArea() < VisionConstants.END_OF_LINE;
    }
}
