// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.ServoConstants;
import frc.robot.IO.JoystickIO;
import frc.robot.subsystems.RomiServo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ServoCommand extends CommandBase {

  private final RomiServo m_servo;
  private final JoystickIO m_joystick;

  /* Creates a new command which controls the romi_servo via 
   * a Joystick.
   * 
   * @param romi_servo Servo subsystem
   * @param joystick Joystick to be used to control the romi_servo
   * 
   */
  public ServoCommand(RomiServo servo,
                      JoystickIO joystick) {
    m_servo = servo;
    m_joystick = joystick;
    addRequirements(servo);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(m_joystick.moveServoLeft().get()) {
      m_servo.incrementServo(-ServoConstants.SERVO_INCREMENT);
      System.out.println("servo -" );
    }
    if(m_joystick.moveServoRight().get()) {
      m_servo.incrementServo(ServoConstants.SERVO_INCREMENT);
      System.out.println("servo +" );
    }

    SmartDashboard.putNumber("servo angle", m_servo.getCurrentAngle());
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
