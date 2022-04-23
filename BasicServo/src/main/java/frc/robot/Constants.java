// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveConstants {

        // -------- Physical Constants -----------------
        public static final double kCountsPerRevolution = 1440.0;
        public static final double kWheelDiameterMeters = 0.07;
        public static final double kMetersPerDegree = Math.PI * 0.141 / 360;
        public static final double kTrackwidthMeters = 0.142072613;
        public static final DifferentialDriveKinematics kDriveKinematics =
            new DifferentialDriveKinematics(kTrackwidthMeters);
    
        // Calibration for the right wheel voltage because it's much slower
        // than the left wheel on this robot.
        public static final double rightVoltsGain = 1.094; 
    }

    public final class ExtIOConstants {
        // Port configuration to match physical configuration on
        // Romi board as well as configuration on http://wpilib.local
        public static final int DIO0_PORT = 8;

        // The Romi has the left and right motors set to
        // PWM channels 0 and 1 respectively
        public static final int PWM2_PORT = 2;
        public static final int PWM3_PORT = 3;
        public static final int PWM4_PORT = 4;
    }

    public final class ServoConstants {
        
        public static final double MIN_RANGE = 0.222;
        public static final double MAX_RANGE = 0.722;
        
        // Incremental amount of change for each button press
        // or during the periodic check while holding the button down
        public static final double SERVO_INCREMENT = 1.0;
    }
}
