// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.sensors.RomiGyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Additional imports from romiReference
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.filter.SlewRateLimiter;

public class Drivetrain extends SubsystemBase {

  // The Romi has the left and right motors set to
  // PWM channels 0 and 1 respectively
  private final Spark m_leftMotor = new Spark(0);
  private final Spark m_rightMotor = new Spark(1);

  // The Romi has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder m_leftEncoder = new Encoder(4, 5);
  private final Encoder m_rightEncoder = new Encoder(6, 7);

  // Set up the differential drive controller
  private final DifferentialDrive m_diffDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);

  // Set up the RomiGyro
  private final RomiGyro m_gyro = new RomiGyro();

  // Set up the BuiltInAccelerometer
  private final BuiltInAccelerometer m_accelerometer = new BuiltInAccelerometer();

  // Odometry class for tracking robot pose. Added in Telemetry lab.
  private final DifferentialDriveOdometry m_odometry;

  // Show a field diagram for tracking odometry. Added in Telemetry lab.
  private final Field2d m_field2d = new Field2d();

  // Create a slew rate filter to give more control over the speed from the joystick
  private final SlewRateLimiter m_filter = new SlewRateLimiter(0.8);
  private final SlewRateLimiter m_filter_turn = new SlewRateLimiter(0.5);

  // Used to put telemetry data onto Shuffleboard
  NetworkTableEntry m_leftFFEntry, m_rightFFEntry, m_headingEntry;
  NetworkTableEntry m_leftWheelPositionEntry, m_rightWheelPositionEntry;
  NetworkTableEntry m_odometryXEntry, m_odometryYEntry, m_odometryHeadingEntry;

  // -----------------------------------------------------------
  // Initialization
  // -----------------------------------------------------------

  /*********************************
   * Creates a new Drivetrain.
  **********************************/
  public Drivetrain() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotor.setInverted(true);
    
    // Use inches as unit for encoder distances
    // Switched to meters in the Robot Structure lab.
    m_leftEncoder.setDistancePerPulse((Math.PI * DriveConstants.kWheelDiameterMeters) / DriveConstants.kCountsPerRevolution);
    m_rightEncoder.setDistancePerPulse((Math.PI * DriveConstants.kWheelDiameterMeters) / DriveConstants.kCountsPerRevolution);
    resetEncoders();

    // Setup Odometry and display the Field2d widget.  Added in Telemetry lab.
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
    SmartDashboard.putData("field", m_field2d);    
    
    setupShuffleboard();
  }

  private void setupShuffleboard() {

    // Create a tab for the Drivetrain
    ShuffleboardTab m_driveTab = Shuffleboard.getTab("Drivetrain");
    m_headingEntry = m_driveTab.add("Heading Deg.", getHeading())
        .withWidget(BuiltInWidgets.kGraph)      
        .withSize(3,3)
        .withPosition(0, 0)
        .getEntry();  
    m_leftWheelPositionEntry = m_driveTab.add("Left Wheel Pos.", getLeftDistanceMeters())
        .withWidget(BuiltInWidgets.kGraph)      
        .withSize(3,3)  
        .withPosition(4, 0)
        .getEntry();  
    m_rightWheelPositionEntry = m_driveTab.add("Right Wheel Pos.", getRightDistanceMeters())
        .withWidget(BuiltInWidgets.kGraph)      
        .withSize(3,3)
        .withPosition(7, 0)
        .getEntry(); 
      
    // Place these on the Odometry tab  
    ShuffleboardTab m_odometryTab = Shuffleboard.getTab("Odometry");    
    m_odometryXEntry = m_odometryTab.add("X Odometry", 0)
        .withWidget(BuiltInWidgets.kGraph)            
        .withSize(2,2)
        .withPosition(7, 0)
        .getEntry();
    m_odometryYEntry = m_odometryTab.add("Y Odometry", 0)
        .withWidget(BuiltInWidgets.kGraph)            
        .withSize(2,2)
        .withPosition(9, 0)
        .getEntry();
    m_odometryHeadingEntry = m_odometryTab.add("Heading Odometry", 0)
        .withWidget(BuiltInWidgets.kGraph)            
        .withSize(2,2)
        .withPosition(8, 3)
        .getEntry();     
  }

  // -----------------------------------------------------------
  // Control Input
  // -----------------------------------------------------------
  public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
    m_diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate);
  }

  public void rateLimitedArcadeDrive(double xaxisSpeed, double zaxisRotate) {
    m_diffDrive.arcadeDrive(m_filter.calculate(xaxisSpeed), m_filter_turn.calculate(zaxisRotate));
  }

  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  /** Reset the gyro. */
  public void resetGyro() {
    m_gyro.reset();
  }

  /**
   * Zeroes the heading of the robot
   */
  public void zeroHeading() {
    m_gyro.reset();
  }

  // -----------------------------------------------------------
  // System State
  // -----------------------------------------------------------
  public int getLeftEncoderCount() {
    return m_leftEncoder.get();
  }

  public int getRightEncoderCount() {
    return m_rightEncoder.get();
  }

  /**
   * The distance in meters
   * 
   * Modified in the Basic Robot Structure lab.
   *
   * @return The distance in meters for the left wheel
   */
  public double getLeftDistanceMeters() {
    return m_leftEncoder.getDistance();
  }

   /**
   * The distance in meters
   * 
   * Modified in the Basic Robot Structure lab.
   *
   * @return The distance in meters for the right wheel
   */
  public double getRightDistanceMeters() {
    return m_rightEncoder.getDistance();
  }

   /**
   * The average distance in meters for both wheels.
   * 
   * Modified in the Basic Robot Structure lab.
   *
   * @return The average distance in meters for both wheels
   */
  public double getAverageDistanceMeters() {
    return (getLeftDistanceMeters() + getRightDistanceMeters()) / 2.0;
  }

  /**
   * The acceleration in the X-axis.
   *
   * @return The acceleration of the Romi along the X-axis in Gs
   */
  public double getAccelX() {
    return m_accelerometer.getX();
  }

  /**
   * The acceleration in the Y-axis.
   *
   * @return The acceleration of the Romi along the Y-axis in Gs
   */
  public double getAccelY() {
    return m_accelerometer.getY();
  }

  /**
   * The acceleration in the Z-axis.
   *
   * @return The acceleration of the Romi along the Z-axis in Gs
   */
  public double getAccelZ() {
    return m_accelerometer.getZ();
  }

  /**
   * Current angle of the Romi around the X-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleX() {
    return m_gyro.getAngleX();
  }

  /**
   * Current angle of the Romi around the Y-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleY() {
    return m_gyro.getAngleY();
  }

  /**
   * Current angle of the Romi around the Z-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleZ() {
    return m_gyro.getAngleZ();
  }

  /**
   * Returns the currently estimated pose of the robot.
   * 
   * Added in Telemetry lab.
   * 
   * @return The pose
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the currently estimated pose of the robot.
   * @return The pose
   */
  // public Pose2d getEstimatedPose() {
  //   return m_estimator.getEstimatedPosition();
  // }

  /**
   * Returns the heading of the robot
   * @return The robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * The left wheel speed
   *
   * @return Speed in meters per/sec for the left wheel
   */
  public double getLeftEncoderRate() {
    return m_leftEncoder.getRate();
  }

  /**
   * The right wheel speed
   *
   * @return Speed in meters per/sec for the right wheel
   */
  public double getRightEncoderRate() {
    return m_rightEncoder.getRate();
  }

  /**
   * Returns the current wheel speeds of the robot.
   * @return The current wheel speeds
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
  }

  // -----------------------------------------------------------
  // Process Logic
  // -----------------------------------------------------------
  @Override
  public void periodic() {
    // This is added in the Telemetry tutorial.
    publishTelemetry();
  }

  /**  
   * Publishes telemetry data to the Network Tables for use
   * in Shuffleboard and the Simulator.  
   * 
   * This function is added in the Telemetry lab.
  */
  public void publishTelemetry() {

    // Display the meters per/second for each wheel and the heading
    SmartDashboard.putNumber("Left Encoder Velocity", m_leftEncoder.getRate());
    SmartDashboard.putNumber("Right Encoder Velocity", m_rightEncoder.getRate());
    SmartDashboard.putNumber("Heading", getHeading());

    // Update the odometry in the periodic block
    m_odometry.update(m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
    
    // Offset the pose to start 1.5 meters on the Y axis
    double yPoseOffset = 1.5;
    Pose2d currentPose = getPose();
    Pose2d poseOffset = new Pose2d(currentPose.getX(), 
                                   currentPose.getY() + yPoseOffset, 
                                   currentPose.getRotation());
    // Update the Field2D object (so that we can visualize this in sim)
    m_field2d.setRobotPose(poseOffset);

    // Display the distance travelled for each wheel
    m_leftWheelPositionEntry.setDouble(getLeftDistanceMeters());
    m_rightWheelPositionEntry.setDouble(getRightDistanceMeters()); 
    m_headingEntry.setDouble(getHeading());
  }

}
