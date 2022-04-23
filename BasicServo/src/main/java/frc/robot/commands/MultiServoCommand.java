// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.ServoConstants;
import frc.robot.IO.JoystickIO;
import frc.robot.subsystems.MultiServoSubsystem;
import frc.robot.subsystems.MultiServoSubsystem;
import frc.robot.subsystems.RomiServo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class MultiServoCommand extends CommandBase {

  private final MultiServoSubsystem m_servos;
  private final JoystickIO m_joystick;

  /* Creates a new command which controls the romi_servo via 
   * a Joystick.
   * 
   * @param romi_servo Servo subsystem
   * @param joystick Joystick to be used to control the romi_servo
   * 
   */
  public MultiServoCommand(MultiServoSubsystem servos,
                           JoystickIO joystick) {
    m_servos = servos;
    m_joystick = joystick;
    addRequirements(servos);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // The joystick methods names should be declarative for your application
    // e.g. panLeft/panRight
    if(m_joystick.moveServoLeft().get()) {
      m_servos.setServo3Angle(-ServoConstants.SERVO_INCREMENT);
    }
    if(m_joystick.moveServoRight().get()) {
      m_servos.setServo3Angle(ServoConstants.SERVO_INCREMENT);
    }
    if(m_joystick.moveServoUp().get()) {
      m_servos.setServo4Angle(-ServoConstants.SERVO_INCREMENT);
    }
    if(m_joystick.moveServoDown().get()) {
      m_servos.setServo4Angle(ServoConstants.SERVO_INCREMENT);
    }

    SmartDashboard.putNumber("Servo 3 angle", m_servos.getServo3Angle());
    SmartDashboard.putNumber("Servo 4 angle", m_servos.getServo4Angle());
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
