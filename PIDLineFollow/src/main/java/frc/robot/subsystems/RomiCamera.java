// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RomiCamera extends SubsystemBase {
  // Constants such as camera and target height stored. Change per robot and goal!
  final double CAMERA_HEIGHT_METERS = 0.1;
  final double TARGET_HEIGHT_METERS = 0.0;

  // Angle between horizontal and the camera.
  final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(-10);

  // How far from the target we want to be
  final double GOAL_RANGE_METERS = Units.feetToMeters(0.5);

  // Change this to match the name of your camera
  PhotonCamera m_camera = new PhotonCamera("photonvision");
  private PhotonPipelineResult m_result;
  
  /** 
   * Contructor
   * Creates a new RomiCamera. 
   * */
  public RomiCamera() {}

  public double getYaw() {
    return m_result.getBestTarget().getYaw();
  }

  public double getSkew() {
    return m_result.getBestTarget().getSkew();
  }

  public boolean hasTargets() {
    // var result = m_camera.getLatestResult();
    return m_result.hasTargets();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_result = m_camera.getLatestResult();
  }
}
