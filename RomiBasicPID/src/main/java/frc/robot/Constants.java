// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final double kCountsPerRevolution = 1440.0;
    public static final double kWheelDiameterMeters = 0.07; // 70 mm

    // For distances PID
    public static final double kPDriveVel = 1.2;
    public static final double kIDriveVel = 0.2;
    public static final double kDDriveVel = 0;

    // For turns PID
    public static final double kPTurnVel = 0.03;
    public static final double kITurnVel = 0;
    public static final double kDTurnVel = 0.002;

    // For profiled distances PID
    public static final double kPDriveProfiled = 1.1;
    public static final double kIDriveProfiled = 0.0;
    public static final double kDDriveProfiled = 0.0;

    // For profiled turns PID
    public static final double kPTurnProfiled = 0.05;
    public static final double kITurnProfiled = 0;
    public static final double kDTurnProfiled = 0;

    // Max speed and acceleration of the robot
    public static final double kMaxSpeedMetersPerSecond = 0.44;
    public static final double kMaxAccelMetersPerSecondSquared = 0.44;

    // Drive profile
    public static final TrapezoidProfile.Constraints kTrapezoidProfileConstraints =
    new TrapezoidProfile.Constraints(kMaxSpeedMetersPerSecond,
                                     kMaxAccelMetersPerSecondSquared);


    public static final double kMaxTurnRateDegPerS = 360;
    public static final double kMaxTurnAccelDegPerSSquared = 200;

    // Turn profile                                    
    public static final TrapezoidProfile.Constraints kTrapezoidProfileTurnConstraints =
    new TrapezoidProfile.Constraints(kMaxTurnRateDegPerS,
                                     kMaxTurnAccelDegPerSSquared);                                 
    
    // The linear inertia gain, volts
    public static final double ksVolts = 0.5;
    public static final double ksVoltsLeft = 0.50;
    public static final double ksVoltsRight = 0.44;
    // The linear velocity gain, volts per (meter per second)
    // Increase this if you drive short
    public static final double kvVoltSecondsPerMeter = 1.888;
    public static final double kvVoltSecondsPerMeterLeft = 1.888;
    // public static final double kvVoltSecondsPerMeterLeft = 2.0;
    // public static final double kvVoltSecondsPerMeterRight = 1.892;
    public static final double kvVoltSecondsPerMeterRight = 1.7;
    // The linear acceleration gain, volts per (meter per second squared).
    public static final double kaVoltSecondsSquaredPerMeter = 0.46138;
    
    // Combined left and right volts feedforward
    public static final SimpleMotorFeedforward kFeedForward = 
        new SimpleMotorFeedforward(ksVolts, 
                                    kvVoltSecondsPerMeter, 
                                    kaVoltSecondsSquaredPerMeter);

    // Left and Right motors are very different, so each has its own FF.
    public static final SimpleMotorFeedforward kLeftFeedForward = 
        new SimpleMotorFeedforward(ksVoltsLeft, 
                                    kvVoltSecondsPerMeterLeft, 
                                    kaVoltSecondsSquaredPerMeter);

    public static final SimpleMotorFeedforward kRightFeedForward = 
        new SimpleMotorFeedforward(ksVoltsRight, 
                                    kvVoltSecondsPerMeterRight, 
                                    kaVoltSecondsSquaredPerMeter);
}
