/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class PIDLineFollow extends CommandBase {
    private final Drivetrain m_drive;
    private final Vision m_vision;

    public PIDLineFollow(Drivetrain drive, Vision vision) {
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
        
        double error = VisionConstants.SETPOINT - m_vision.getCenterX();
        double turn = error * DriveConstants.kPDriveVel;
        System.out.println("Error " + error); 
        System.out.println("Turn " + turn);

        // Limit the turn amount
        if (turn > 0.5) {
            turn = 0.5;
            System.out.println("Turn corrected" + turn);
        } 
        if (turn < -0.5) {
            turn = -0.5;
            System.out.println("Turn corrected" + turn);
        }
        m_drive.arcadeDrive(0.5, turn);        
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.arcadeDrive(0, 0);
    }

    @Override
    public boolean isFinished() {
        return m_vision.getRectHeight() < VisionConstants.END_OF_LINE;
    }
}
