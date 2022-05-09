// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.sensors.RomiGyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;

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

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  // Show a field diagram for tracking odometry
  private final Field2d m_field2d = new Field2d();

  // Pose estimator State Space way for tracking the robot pose
  private DifferentialDrivePoseEstimator m_estimator = new DifferentialDrivePoseEstimator(new Rotation2d(), new Pose2d(),
        new MatBuilder<>(Nat.N5(), Nat.N1()).fill(0.02, 0.02, 0.01, 0.02, 0.02), // State measurement standard deviations. X, Y, theta.
        new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.01), // Local measurement standard deviations. Left encoder, right encoder, gyro.
        new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1, 0.1, 0.01)); // Global measurement standard deviations. X, Y, and theta.

  // Show a field diagram for tracking Pose estimation
  private final Field2d m_estimatedField2d = new Field2d();

  // Create a slew rate filter to give more control over the speed from the joystick
  private final SlewRateLimiter m_filter = new SlewRateLimiter(0.5);
  private final SlewRateLimiter m_filter_turn = new SlewRateLimiter(0.5);

  // Used to put data onto Shuffleboard
  private ShuffleboardTab driveTab = Shuffleboard.getTab("Drivetrain");

  private NetworkTableEntry m_leftVolts = 
    driveTab.add("Left Volts", 0)
      .withWidget(BuiltInWidgets.kGraph)
      .withPosition(3, 3)
      .getEntry(); 

  private NetworkTableEntry m_rightVolts = 
    driveTab.add("Right Volts", 0)
      .withWidget(BuiltInWidgets.kGraph)
      .withPosition(3, 3)
      .getEntry();     

  // -----------------------------------------------------------
  // Initialization
  // -----------------------------------------------------------

  /** Contructor
   * Creates a new Drivetrain. */
  public Drivetrain() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotor.setInverted(true);
    
    // Use inches as unit for encoder distances
    m_leftEncoder.setDistancePerPulse((Math.PI * DrivetrainConstants.kWheelDiameterMeters) / DrivetrainConstants.kCountsPerRevolution);
    m_rightEncoder.setDistancePerPulse((Math.PI * DrivetrainConstants.kWheelDiameterMeters) / DrivetrainConstants.kCountsPerRevolution);
    resetEncoders();

    Pose2d initialPose = new Pose2d(0, 1.5, m_gyro.getRotation2d()); 
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d(), initialPose);
    m_field2d.setRobotPose(initialPose);
    SmartDashboard.putData("field", m_field2d);
    SmartDashboard.putData("fieldEstimate", m_estimatedField2d);
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

  /**
   * Controls the left and right sides of the drive directly with voltages.
   * 
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    
    double rightVoltsCalibrated = rightVolts * DrivetrainConstants.rightVoltsGain;

    // Send to Network Tables
    m_leftVolts.setDouble(leftVolts);
    m_rightVolts.setDouble(rightVoltsCalibrated);

    // Apply the voltage to the wheels
    m_leftMotor.setVoltage(leftVolts);
    m_rightMotor.setVoltage(-rightVoltsCalibrated); // We invert this to maintain +ve = forward
    m_diffDrive.feed();
  }

  public void stopDrivetrain() {
    tankDriveVolts(0, 0);
  }

  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  /**
   * Resets the odometry to the specified pose
   * @param pose The pose to which to set the odometry
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly
   * @param maxOutput The maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_diffDrive.setMaxOutput(maxOutput);
  }

  /**
   * Zeroes the heading of the robot
   */
  public void zeroHeading() {
    m_gyro.reset();
  }
  
  // -----------------------------------------------------------
  // Process Logic
  // -----------------------------------------------------------
  @Override
  public void periodic() {
    publishTelemetry();
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
   * @return The distance in meters for the left wheel
   */
  public double getLeftDistanceMeters() {
    return m_leftEncoder.getDistance();
  }

   /**
   * The distance in meters
   *
   * @return The distance in meters for the right wheel
   */
  public double getRightDistanceMeters() {
    return m_rightEncoder.getDistance();
  }

   /**
   * The average distance in meters for both wheels
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

  /** Reset the gyro. */
  public void resetGyro() {
    m_gyro.reset();
  }

  /**  
   * Publishes telemetry data to the Network Tables for use
   * in Shuffleboard and the Simulator
  */
  public void publishTelemetry() {
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

    // Updates the the Unscented Kalman Filter using only wheel encoder information.
    m_estimator.update(m_gyro.getRotation2d(), 
                      getWheelSpeeds(), 
                       m_leftEncoder.getDistance(), 
                       m_rightEncoder.getDistance());


    // Offset the pose to start 1.5 meters on the Y axis
    Pose2d currentEstimatedPose = getEstimatedPose();
    Pose2d estimatedPoseOffset = new Pose2d(currentEstimatedPose.getX(), 
                                            currentEstimatedPose.getY() + yPoseOffset, 
                                            currentEstimatedPose.getRotation());

    // Update the Field2D object (so that we can visualize this in sim)
    m_estimatedField2d.setRobotPose(estimatedPoseOffset);

    // Display the meters per/second for each wheel and the heading
    SmartDashboard.putNumber("Left Encoder Velocity", m_leftEncoder.getRate());
    SmartDashboard.putNumber("Right Encoder Velocity", m_rightEncoder.getRate());
    SmartDashboard.putNumber("Heading", getHeading());
  }

  /**
   * Returns the currently estimated pose of the robot.
   * @return The pose
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the currently estimated pose of the robot.
   * @return The pose
   */
  public Pose2d getEstimatedPose() {
    return m_estimator.getEstimatedPosition();
  }

  /**
   * Returns the current wheel speeds of the robot.
   * @return The current wheel speeds
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
  }

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
   * Returns the turn rate of the robot
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -m_gyro.getRate();
  }
}
