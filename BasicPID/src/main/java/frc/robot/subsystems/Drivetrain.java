// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.sensors.RomiGyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Additional imports from romiReference
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DrivetrainConstants;

import edu.wpi.first.math.filter.SlewRateLimiter;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;

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

  // Also show a field diagram
  private final Field2d m_field2d = new Field2d();

  // Create a slew rate filter to give more control over the speed from the joystick
  private final SlewRateLimiter m_filter = new SlewRateLimiter(0.5);
  private final SlewRateLimiter m_filter_turn = new SlewRateLimiter(0.5);
  
  // Used to put telemetry data onto Shuffleboard
  NetworkTableEntry m_leftFFEntry, m_rightFFEntry, m_headingEntry;
  NetworkTableEntry m_leftWheelPositionEntry, m_rightWheelPositionEntry;
  NetworkTableEntry m_odometryXEntry, m_odometryYEntry, m_odometryHeadingEntry;

  // PID Parameters
  NetworkTableEntry m_distance;
  NetworkTableEntry m_distanceP, m_distanceD, m_distanceI;
  NetworkTableEntry m_angle, m_angleP, m_angleD;

  private final PIDController m_leftController =
    new PIDController(DrivetrainConstants.kPDriveVel, 
                      DrivetrainConstants.kIDriveVel, 
                      DrivetrainConstants.kDDriveVel);

  private final PIDController m_rightController =
  new PIDController(DrivetrainConstants.kPDriveVel, 
                    DrivetrainConstants.kIDriveVel, 
                    DrivetrainConstants.kDDriveVel);    

  /*********************************
   * Creates a new Drivetrain.
   * 
  */
  public Drivetrain() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotor.setInverted(true);
    
    // Use inches as unit for encoder distances
    m_leftEncoder.setDistancePerPulse((Math.PI * DrivetrainConstants.kWheelDiameterMeters) / DrivetrainConstants.kCountsPerRevolution);
    m_rightEncoder.setDistancePerPulse((Math.PI * DrivetrainConstants.kWheelDiameterMeters) / DrivetrainConstants.kCountsPerRevolution);
    
    // Reset pose
    resetEncoders();
    resetGyro();

    // Call these to so that the information is immediatelly available in Simulator
    arcadeDrive(0, 0);
    turn(0);

    // Setup Odometry
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

    // Add entries to display the Feedforward.    
    m_leftFFEntry=m_driveTab.add("Left FF", 0)
        .withWidget(BuiltInWidgets.kGraph)      
        .withSize(3,3)            
        .withPosition(4, 3)
        .getEntry();  
    m_rightFFEntry=m_driveTab.add("Right FF", 0)
        .withWidget(BuiltInWidgets.kGraph)            
        .withSize(3,3)
        .withPosition(7, 3)
        .getEntry();   
        
    // Add PID tuning parameters (distance)
    m_distanceP = m_driveTab.add("kP", DrivetrainConstants.kPDriveVel)
      .withPosition(0, 3)
      .getEntry();  

    m_distanceD = m_driveTab.add("kD", DrivetrainConstants.kDDriveVel)
      .withPosition(0, 4)
      .getEntry();  

    m_distanceI = m_driveTab.add("kI", DrivetrainConstants.kIDriveVel)
      .withPosition(0, 5)
      .getEntry();    
  
    // Add PID tuning parameters (turning)
    m_angleP = m_driveTab.add("anglekP", DrivetrainConstants.kPTurnVel)
      .withPosition(1, 3)
      .getEntry();  

    m_angleD = m_driveTab.add("anglekD", DrivetrainConstants.kDTurnVel)
      .withPosition(1, 4)
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
  public void drive(DoubleSupplier move, DoubleSupplier rotate){
    arcadeDrive(move.getAsDouble(), rotate.getAsDouble());
  }

  public void drive(double move, double rotate){
    arcadeDrive(move, rotate);
  }

  public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
    SmartDashboard.putNumber("ArcadeDrive xaxisSpeed", xaxisSpeed);
    SmartDashboard.putNumber("ArcadeDrive zaxisRotate", zaxisRotate);
    m_diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate);
  }

  public void rateLimitedArcadeDrive(double xaxisSpeed, double zaxisRotate) {
    m_diffDrive.arcadeDrive(m_filter.calculate(xaxisSpeed), m_filter_turn.calculate(zaxisRotate));
  }

  /**
   * Tank drive method for differential drive platform. The calculated values 
   * will be squared to decrease sensitivity at low speeds.
   *
   * @param leftSpeed The robot's left side speed along the X axis [-1.0..1.0].
   * @param rightSpeed The robot's right side speed along the X axis [-1.0..1.0].
   */
  public void tankDrive(double leftSpeed, double rightSpeed) {
    SmartDashboard.putNumber("PID Left Output", leftSpeed);
    SmartDashboard.putNumber("PID Right Output", rightSpeed);
    
    m_diffDrive.tankDrive(leftSpeed, rightSpeed);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   * 
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    
    // double rightVoltsCalibrated = rightVolts * DrivetrainConstants.rightVoltsGain;
    SmartDashboard.putNumber("Total Left Volts", leftVolts);
    SmartDashboard.putNumber("Total Right Volts", rightVolts);

    // Apply the voltage to the wheels
    m_leftMotor.setVoltage(leftVolts);
    m_rightMotor.setVoltage(-rightVolts); // We invert this to maintain +ve = forward
    m_diffDrive.feed();
  }

  /**
   * Drives a straight line at the requested velocity by applying feedforward
   * and PID output to maintain the velocity. This method calculates a voltage
   * value for each wheel, which is sent to the motors setVoltage() method.
   * 
   * @param velocity The velocity at which to drive
   */
  public void setOutputMetersPerSecond(double velocity) {
    SmartDashboard.putNumber("Requested Velocity", velocity);

    // Calculate feedforward voltage
    double leftFeedforward = DrivetrainConstants.kLeftFeedForward.calculate(velocity);
    double rightFeedforward = DrivetrainConstants.kRightFeedForward.calculate(velocity);
    SmartDashboard.putNumber("Left Feedforward Volts", leftFeedforward);
    SmartDashboard.putNumber("Right Feedforward Volts", rightFeedforward);

    // Send it through a PID controller
    double leftVelocity = m_leftController.calculate(m_leftEncoder.getRate(), velocity);
    double rightVelocity = m_rightController.calculate(m_rightEncoder.getRate(), velocity);
    SmartDashboard.putNumber("Left Volts", leftVelocity);
    SmartDashboard.putNumber("Right Volts", rightVelocity);

    // double calibratedRightSpeed = output * DrivetrainConstants.rightVoltsGain;
    tankDriveVolts(leftFeedforward + leftVelocity, rightFeedforward + rightVelocity);
  }

  // /**
  //  * Drives a straight line at the requested output -1.0..1.0
  //  * 
  //  * @param output Output value between -1.0..1.0
  //  */
  // public void steer(double output) {
  //   SmartDashboard.putNumber("Requested Output", output);

  //   // double calibratedRightSpeed = output * DrivetrainConstants.rightVoltsGain;
  //   tankDrive(output, output);
  // }

  public void turn(double output) {
    // Restrict the turn speed
    double zRotation = MathUtil.clamp(output, -0.5, 5.0);
    arcadeDrive(0, zRotation); 
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
   * Zeroes the heading of the robot
   */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /** Reset the gyro. */
  public void resetGyro() {
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

  public double getLeftDistanceMeters() {
    return m_leftEncoder.getDistance();
  }

  public double getRightDistanceMeters() {
    return m_rightEncoder.getDistance();
  }

  public double getAverageDistanceMeters() {
    double leftDistance = getLeftDistanceMeters();
    double rightDistance = getRightDistanceMeters();
    SmartDashboard.putNumber("Left distance", leftDistance);
    SmartDashboard.putNumber("Right distance", rightDistance);
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
   * @return The pose
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public double getHeading() {
    
    // double angle = getGyroAngleZ();
    // double rotations = Math.round(angle/360);
    // double heading = angle - (rotations*360);
    double heading = m_gyro.getRotation2d().getDegrees();
    return heading;
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
    publishTelemetry();
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

    // Display the meters per/second for each wheel and the heading
    SmartDashboard.putNumber("Left Encoder Velocity", m_leftEncoder.getRate());
    SmartDashboard.putNumber("Right Encoder Velocity", m_rightEncoder.getRate());
    SmartDashboard.putNumber("Heading", getHeading());
  }

}
