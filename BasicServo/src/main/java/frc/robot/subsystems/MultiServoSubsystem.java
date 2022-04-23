// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ExtIOConstants;

public class MultiServoSubsystem extends SubsystemBase {
  

  // Default position is 90 degrees
  private final RomiServo m_servo3 = new RomiServo(ExtIOConstants.PWM3_PORT);
  private final RomiServo m_servo4 = new RomiServo(ExtIOConstants.PWM4_PORT);

  /** Constructor
   * Creates a new MultiServoSubsystem. 
   * 
  */
  public MultiServoSubsystem() {
    m_servo3.setDefaultAngle(90);
    m_servo4.setDefaultAngle(90);
  }

  /**
   * Move servo
   * 
   * @param delta Amount to change servo position (degrees)
   */
  public void setServo4Angle(double delta) {
    m_servo4.incrementServo(delta);
  }

  /**
   * Move servo 
   * 
   * @param delta Amount to change servo position (degrees)
   */
  public void setServo3Angle(double delta) {
    m_servo3.incrementServo(delta);
  }

  /**
   * 
   * @return The current servo angle
   */
  public double getServo4Angle() {
    return m_servo4.getCurrentAngle();
  }

  /**
   * 
   * @return The current servo angle
   */
  public double getServo3Angle() {
    return m_servo3.getCurrentAngle();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
