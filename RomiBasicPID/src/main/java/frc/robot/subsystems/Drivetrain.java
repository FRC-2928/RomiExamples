// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants;
import frc.robot.sensors.RomiGyro;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

  // Used to put telemetry data onto Shuffleboard
  NetworkTableEntry m_headingEntry, m_avgDistanceEntry;
  NetworkTableEntry m_leftWheelSpeedsEntry, m_rightWheelSpeedsEntry;
  NetworkTableEntry m_leftWheelPositionEntry, m_rightWheelPositionEntry;
  NetworkTableEntry m_distanceP, m_distanceD, m_distanceI;
  NetworkTableEntry m_driveProfiledP, m_driveProfiledD, m_driveProfiledI;
  NetworkTableEntry m_angleI, m_angleP, m_angleD;

  private final PIDController m_leftController =
    new PIDController(1.2, 0.0, 0.0);

  private final PIDController m_rightController =
    new PIDController(1.2, 0.0, 0.0);    
  
  // private double m_leftLastDistance = 0.0;
  // private double m_rightLastDistance = 0.0;

  // -----------------------------------------------------------
  // Initialization
  // -----------------------------------------------------------

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotor.setInverted(true);

    // Use inches as unit for encoder distances
    m_leftEncoder.setDistancePerPulse((Math.PI * Constants.kWheelDiameterMeters) / Constants.kCountsPerRevolution);
    m_rightEncoder.setDistancePerPulse((Math.PI * Constants.kWheelDiameterMeters) / Constants.kCountsPerRevolution);
    resetEncoders();
    setupShuffleboard();
  }

  private void setupShuffleboard() {

    // Create a tab for the Drivetrain
    ShuffleboardTab m_driveTab = Shuffleboard.getTab("Drivetrain");

    // Add basic telemetry metrics
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
    m_avgDistanceEntry = m_driveTab.add("Average Distance", getAverageDistanceMeters())
        .withWidget(BuiltInWidgets.kGraph)      
        .withSize(3,3)
        .withPosition(10, 0)
        .getEntry();
    // m_leftWheelSpeedsEntry = m_driveTab.add("Left Wheel Speed", getWheelSpeeds().leftMetersPerSecond)
    //     .withWidget(BuiltInWidgets.kGraph)      
    //     .withSize(3,3)
    //     .withPosition(13, 0)
    //     .getEntry();
    // m_rightWheelSpeedsEntry = m_driveTab.add("Right Wheel S", getRightRate())
    //     .withWidget(BuiltInWidgets.kGraph)      
    //     .withSize(3,3)
    //     .withPosition(15, 0)
    //     .getEntry();             
        
    // Add PID tuning parameters (distance)
    m_distanceP = m_driveTab.add("DistancekP", Constants.kPDriveVel)
      .withPosition(0, 3)
      .getEntry();  

    m_distanceI = m_driveTab.add("DistancekI", Constants.kIDriveVel)
      .withPosition(0, 4)
      .getEntry();  

    m_distanceD = m_driveTab.add("DistancekD", Constants.kDDriveVel)
      .withPosition(0, 5)
      .getEntry();    
      
    // Add PID tuning parameters (turning)
    m_angleP = m_driveTab.add("TurnkP", Constants.kPTurnVel)
      .withPosition(1, 3)
      .getEntry();  

    m_angleI = m_driveTab.add("TurnkI", Constants.kITurnVel)
      .withPosition(1, 4)
      .getEntry(); 

    m_angleD = m_driveTab.add("TurnkD", Constants.kDTurnVel)
      .withPosition(1, 5)
      .getEntry(); 
      
    // Add Profiled PID tuning parameters (distance)
    m_driveProfiledP = m_driveTab.add("DriveProfiledkP", Constants.kPDriveProfiled)
      .withPosition(2, 3)
      .getEntry();  

    m_driveProfiledI = m_driveTab.add("DriveProfiledkI", Constants.kIDriveProfiled)
      .withPosition(2, 4)
      .getEntry(); 

    m_driveProfiledD = m_driveTab.add("DriveProfilekD", Constants.kDDriveProfiled)
      .withPosition(2, 5)
      .getEntry();    
  }

  // -----------------------------------------------------------
  // Control Input
  // -----------------------------------------------------------
  public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
    m_diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate);
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
   * Controls the left and right sides of the drive directly with voltages.
   * 
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {

    SmartDashboard.putNumber("Left Volts", leftVolts);
    SmartDashboard.putNumber("Right Volts", rightVolts);

    // Apply the voltage to the wheels
    m_leftMotor.setVoltage(leftVolts);
    m_rightMotor.setVoltage(rightVolts); 
    m_diffDrive.feed();
  }

  /**
   * Drives a straight line at the requested velocity by applying feedforward
   * and PID output to maintain the velocity. This method calculates a voltage
   * value for each wheel, which is sent to the motors setVoltage() method.
   * 
   * @param velocity The velocity at which to drive
   */
  public void setOutputMetersPerSecond(double velocity, double setpointVel, double setpointPos) {
    
    SmartDashboard.putNumber("Requested Velocity", velocity);
    SmartDashboard.putNumber("Setpoint Velocity", setpointVel);
    SmartDashboard.putNumber("Setpoint Position", setpointPos);

    // Calculate feedforward voltage
    double leftFeedforward = Constants.kLeftFeedForward.calculate(velocity);
    double rightFeedforward = Constants.kRightFeedForward.calculate(velocity);
    SmartDashboard.putNumber("Left Feedforward Volts", leftFeedforward);
    SmartDashboard.putNumber("Right Feedforward Volts", rightFeedforward);
  
    // Send it through a PID controller
    double leftPIDVolts = m_leftController.calculate(m_leftEncoder.getRate(), velocity);
    double rightPIDVolts = m_rightController.calculate(m_rightEncoder.getRate(), velocity);
    SmartDashboard.putNumber("Left PID Volts", leftPIDVolts);
    SmartDashboard.putNumber("Right PID Volts", rightPIDVolts);
    
    // Add the voltage values and send them to the motors
    tankDriveVolts(leftFeedforward + leftPIDVolts, rightFeedforward + rightPIDVolts);
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
    return (getLeftDistanceMeters() + getRightDistanceMeters()) / 2.0;
  }

  // /**
  //  * Returns the current left wheel speeds of the robot in meters per second.
  //  * 
  //  * @return The current left wheel speed
  //  */
  // public double getLeftRate() {
  //   double rate =  getLeftDistanceMeters() - m_leftLastDistance;  
  //   m_leftLastDistance = getLeftDistanceMeters();    
  //   return rate * 50; // Use 50 cycles per second
  // }

  // /**
  //  * Returns the current right wheel speeds of the robot in meters per second.
  //  * 
  //  * @return The current right wheel speed
  //  */
  // public double getRightRate() {
  //   double rate =  getRightDistanceMeters() - m_rightLastDistance;    
  //   m_rightLastDistance = getRightDistanceMeters(); 
  //   return rate * 50; // Use 50 cycles per second 
  // }

  /**
   * Returns the current wheel speeds of the robot.
   * 
   * @return The current wheel speeds
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
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

  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }
  
  // -----------------------------------------------------------
  // Process Logic
  // -----------------------------------------------------------
  @Override
  public void periodic() {
    publishTelemetry();
  }

  public void publishTelemetry() {
        
    // Display the meters per/second for each wheel and the heading
    DifferentialDriveWheelSpeeds wheel_speeds = getWheelSpeeds();
    SmartDashboard.putNumber("Left Wheel Speed", wheel_speeds.leftMetersPerSecond);
    SmartDashboard.putNumber("Right Wheel Speed", wheel_speeds.rightMetersPerSecond);
    // SmartDashboard.putNumber("Left Wheel Speed", m_leftEncoder.getRate());
    // SmartDashboard.putNumber("Right Wheel Speed", m_rightEncoder.getRate());
    SmartDashboard.putNumber("Heading", getHeading());
    

    // Display the distance travelled for each wheel
    m_leftWheelPositionEntry.setDouble(getLeftDistanceMeters());
    m_rightWheelPositionEntry.setDouble(getRightDistanceMeters()); 
    m_avgDistanceEntry.setDouble(getAverageDistanceMeters());
    m_headingEntry.setDouble(getHeading());
    // m_leftWheelSpeedsEntry.setDouble(getWheelSpeeds().leftMetersPerSecond);
    // m_rightWheelSpeedsEntry.setDouble(getRightRate());
  }

}
