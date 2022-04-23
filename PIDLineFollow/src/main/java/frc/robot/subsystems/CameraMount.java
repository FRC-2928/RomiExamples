// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ExtIOConstants;

public class CameraMount extends SubsystemBase {
  

  // Default position is 90 degrees
  private final RomiServo m_pan = new RomiServo(ExtIOConstants.PWM4_PORT);
  private final RomiServo m_tilt = new RomiServo(ExtIOConstants.PWM3_PORT);

  /** Constructor
   * Creates a new CameraMount. 
   * 
  */
  public CameraMount() {
    m_pan.setDefaultAngle(98);
  }

  /**
   * Pan the camera mount left and right
   * 
   * @param delta Amount to change servo position (degrees)
   */
  public void pan(double delta) {
    m_pan.incrementServo(delta);
  }

  /**
   * Tilt the camera mount left and right
   * 
   * @param delta Amount to change servo position (degrees)
   */
  public void tilt(double delta) {
    m_tilt.incrementServo(delta);
  }

  /**
   * 
   * @return The current camera mount pan angle
   */
  public double getPanAngle() {
    return m_pan.getCurrentAngle();
  }

  /**
   * 
   * @return The current camera mount tilt angle
   */
  public double getTiltAngle() {
    return m_tilt.getCurrentAngle();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
