// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.IO;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/** Add your docs here. */
public class JoystickIO {
  private XboxController m_controller;

  public JoystickIO(XboxController controller) {
    m_controller = controller;
  }

  public Button panLeft() {
    return new JoystickButton(m_controller, XboxController.Button.kX.value);
  }

  public Button panRight() {
    return new JoystickButton(m_controller, XboxController.Button.kB.value);
  }

  public Button tiltUp() {
    return new JoystickButton(m_controller, XboxController.Button.kY.value);
  }

  public Button tiltDown() {
    return new JoystickButton(m_controller, XboxController.Button.kA.value);
  }


}
