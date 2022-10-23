// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DrivetrainConstants {

        // -------- Physical Constants -----------------
        public static final double kCountsPerRevolution = 1440.0;
        public static final double kWheelDiameterMeters = 0.07; // 70 mm
        public static final double kTrackwidthMeters = 0.142072613;
        public static final double kMetersPerDegree = Math.PI * 0.141 / 360;
        public static final DifferentialDriveKinematics kDriveKinematics =
            new DifferentialDriveKinematics(kTrackwidthMeters);

        // Calibration for the right wheel voltage because it's much slower
        // than the left wheel on this robot.
        public static final double rightVoltsGain = 1.03;

        // -------- Dynamical Constants --------------------

        // Max speed and acceleration of the robot
        public static final double kMaxSpeedMetersPerSecond = 0.5;
        public static final double kMaxAccelMetersPerSecondSquared = 0.5;

        public static final double kMaxTurnRateDegPerS = 360;
        public static final double kMaxTurnAccelDegPerSSquared = 200;

        // The linear inertia gain, volts
        public static final double ksVolts = 0.461;
        // The linear velocity gain, volts per (meter per second)
        // Increase this if you drive short
        public static final double kvVoltSecondsPerMeter = 6.93;
        public static final double kvVoltSecondsPerMeterLeft = 8.8;
        public static final double kvVoltSecondsPerMeterRight = 9.5;
        // The linear acceleration gain, volts per (meter per second squared).
        public static final double kaVoltSecondsSquaredPerMeter = 0.0737;

        // Setup constraints for feedforward and kinematics
        public static final SimpleMotorFeedforward kFeedForward = 
            new SimpleMotorFeedforward(ksVolts, 
                                        kvVoltSecondsPerMeter, 
                                        kaVoltSecondsSquaredPerMeter);
                                        
        // Left and Right motors are very different, so each has its own FF.
        public static final SimpleMotorFeedforward kLeftFeedForward = 
            new SimpleMotorFeedforward(ksVolts, 
                                        kvVoltSecondsPerMeterLeft, 
                                        kaVoltSecondsSquaredPerMeter);

        public static final SimpleMotorFeedforward kRightFeedForward = 
            new SimpleMotorFeedforward(ksVolts, 
                                        kvVoltSecondsPerMeterRight, 
                                        kaVoltSecondsSquaredPerMeter);

        public static final DifferentialDriveVoltageConstraint kAutoVoltageConstraint =
            new DifferentialDriveVoltageConstraint(
                kFeedForward,
                kDriveKinematics,
                10);

        // -------- PID Constants --------------------

        // Drive profile
        public static final TrapezoidProfile.Constraints kTrapezoidProfileConstraints =
            new TrapezoidProfile.Constraints(DrivetrainConstants.kMaxSpeedMetersPerSecond,
                                            DrivetrainConstants.kMaxAccelMetersPerSecondSquared);

        // Turn profile                                    
        public static final TrapezoidProfile.Constraints kTrapezoidProfileTurnConstraints =
            new TrapezoidProfile.Constraints(DrivetrainConstants.kMaxTurnRateDegPerS,
                                             DrivetrainConstants.kMaxTurnAccelDegPerSSquared);                                    

        // For distances PID
        public static final double kPDriveVel = 1.1;
        public static final double kIDriveVel = 0.2;
        public static final double kDDriveVel = 0;

        // Tolerances for distance drive
        public static final double kDistanceToleranceMeters = 0.06;
        public static final double kVelocityToleranceMetersPerS = 0.05;

        // For turns PID
        public static final double kPTurnVel = 0.05;
        public static final double kITurnVel = 0;
        public static final double kDTurnVel = 0.005;

        // For profiled turns PID
        public static final double kPTurnVelProfiled = 0.05;
        public static final double kITurnVelProfiled = 0;
        public static final double kDTurnVelProfiled = 0;

        // Tolerances for turns
        public static final double kTurnToleranceDeg = 5.0;
        public static final double kTurnRateToleranceDegPerS = 10.0;
    }
}
