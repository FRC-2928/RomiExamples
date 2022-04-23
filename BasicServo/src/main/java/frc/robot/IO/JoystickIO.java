// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.IO;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/** Add your docs here. */
public class JoystickIO {

  public final class Joystick {
    // Button mapping for a Logitech/PS3 gamepad. 
    public static final int X = 1;
    public static final int A = 2;
    public static final int B = 3;
    public static final int Y = 4;
    public static final int TOPLEFT = 5;
    public static final int TOPRIGHT = 6;
    public static final int BOTTOMLEFT = 7;
    public static final int BOTTOMRIGHT = 8;
    public static final int BACK = 9;
    public static final int START = 10;
    public static final int LEFT_ANALOG = 11;
    public static final int RIGHT_ANALOG = 12;
  }

  private GenericHID m_controller;

  /**
   * Constructor
   * 
   * @param controller Generic controller being used, e.g. Xbox, Logitech, PS3
   * 
   */
  public JoystickIO(GenericHID controller) {
    m_controller = controller;
  }

  // The joystick methods names should be declarative for your application
  // e.g. panCameraLeft/Right, moveElevatorUp/Down

  public double xAxisSpeed() {
    return m_controller.getRawAxis(1);
  }

  public double zAxisRotate() {
    if (m_controller.getClass() == XboxController.class) {
      return m_controller.getRawAxis(4);
    } else {
      return m_controller.getRawAxis(2);
    }       
  }

  public double xAxisBoostSpeed() {
    // Default to PS3
    double boostValue = mapRange(-1, 1, 0, 1, m_controller.getRawAxis(3));
    double mainAxisPercent = 0.75;

    if (m_controller.getClass() == XboxController.class) {
      boostValue = m_controller.getRawAxis(2);    
    }
    return xAxisSpeed() * (mainAxisPercent + (1-mainAxisPercent) * boostValue);
  }

  public double zAxisBoostRotate() {
    // Default to PS3 
    double boostValue = mapRange(-1, 1, 0, 1, m_controller.getRawAxis(4));
    double mainAxisPercent = 0.6;

    if (m_controller.getClass() == XboxController.class) {
      boostValue = m_controller.getRawAxis(3);
    }
    double boostPct = mainAxisPercent + (1-mainAxisPercent) * boostValue;
    SmartDashboard.putNumber("boostValue", boostValue);
    SmartDashboard.putNumber("boostPct", boostPct);
    return zAxisRotate() * boostPct;
  }

  public Button moveServoLeft() {
    if (m_controller.getClass() == XboxController.class) {
      return new JoystickButton(m_controller, XboxController.Button.kX.value);
    } else {
      return new JoystickButton(m_controller, Joystick.X);
    }   
  }

  public Button moveServoRight() {
    if (m_controller.getClass() == XboxController.class) {
      return new JoystickButton(m_controller, XboxController.Button.kB.value);
    } else {
      return new JoystickButton(m_controller, Joystick.B);
    } 
  }

  public Button moveServoUp() {
    if (m_controller.getClass() == XboxController.class) {
      return new JoystickButton(m_controller, XboxController.Button.kY.value);
    } else {
      return new JoystickButton(m_controller, Joystick.Y);
    } 
  }

  public Button moveServoDown() {
    if (m_controller.getClass() == XboxController.class) {
      return new JoystickButton(m_controller, XboxController.Button.kA.value);
    } else {
      return new JoystickButton(m_controller, Joystick.A);
    } 
  }

  /**
   * Utility function to remap a range of values
   * 
   * @param a1 Low end of range to map from 
   * @param a2 High end of range to map from
   * @param b1 Low end of range to map to
   * @param b2 High end of range to map to
   * @param s  The value that you want to map
   * @return The remapped value
   */
  public double mapRange(double a1, double a2, double b1, double b2, double s){
    return b1 + ((s - a1)*(b2 - b1))/(a2 - a1);
  }

}
