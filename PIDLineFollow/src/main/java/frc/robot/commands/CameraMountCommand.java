// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.ServoConstants;
import frc.robot.IO.JoystickIO;
import frc.robot.subsystems.CameraMount;
import frc.robot.subsystems.RomiServo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class CameraMountCommand extends CommandBase {

  private final CameraMount m_camera_mount;
  private final JoystickIO m_joystick;

  /* Creates a new command which controls the romi_servo via 
   * a Joystick.
   * 
   * @param romi_servo Servo subsystem
   * @param joystick Joystick to be used to control the romi_servo
   * 
   */
  public CameraMountCommand(CameraMount cameraMount,
                            JoystickIO joystick) {
    m_camera_mount = cameraMount;
    m_joystick = joystick;
    addRequirements(cameraMount);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Put it to default pan when we connect to the microcontroller.
    m_camera_mount.pan(ServoConstants.SERVO_INCREMENT);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(m_joystick.panLeft().get()) {
      m_camera_mount.pan(-ServoConstants.SERVO_INCREMENT);
      System.out.println("Pan left" );
    }
    if(m_joystick.panRight().get()) {
      m_camera_mount.pan(ServoConstants.SERVO_INCREMENT);
      System.out.println("Pan right" );
    }
    if(m_joystick.tiltUp().get()) {
      m_camera_mount.tilt(-ServoConstants.SERVO_INCREMENT);
      System.out.println("Tilt up" );
    }
    if(m_joystick.tiltDown().get()) {
      m_camera_mount.tilt(ServoConstants.SERVO_INCREMENT);
      System.out.println("Tilt down" );
    }

    SmartDashboard.putNumber("Camera Pan angle", m_camera_mount.getPanAngle());
    SmartDashboard.putNumber("Camera Tilt angle", m_camera_mount.getTiltAngle());
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
