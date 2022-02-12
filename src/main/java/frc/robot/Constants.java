// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

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
public class Constants {


    public final static class DriveConstants {

        public static final double kMaxDrivetrainVolts = 11.0;


        public static final int kLeftMotor1Port = 11;
        public static final int kLeftMotor2Port = 12;

        public static final int kRightMotor1Port = 13;
        public static final int kRightMotor2Port = 14;


        public static final double ksVolts = 0.649;
        public static final double kvVoltSecondsPerMeter = 0.866;
        public static final double kaVoltSecondsSquaredPerMeter = 0.126;

        public static final double kPDriveVel = 2.65;

        public static final double kTrackwidthMeters = 0.51;
        
        // 0.15 meters - diameter of the wheel; 2048 units per rotation - Talon FX; 8.45:1 Gearbox Ratio;
        public static final double kMetersRobotTravelPerEncoderCount = .15 * 3.14 / (2048 * 8.45);   

        // Meters of Robot Travel per second per encoder tick per 100 milliseconds; multiplied by 10 to convert to seconds.
        public static final double kMetersRobotTravelPerSecondPerTalonUnits = kMetersRobotTravelPerEncoderCount * 10; 
        
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
                kTrackwidthMeters);

    }
        public static final class OIConstants {
            public static final int kDriverControllerPort = 0;
            public static final double kExtreme3DProDeadband = 0.02;
        }
        
        public static final class AutoConstants {
            public static final double kMaxSpeedMetersPerSecond = 1;
            public static final double kMaxAccelerationMetersPerSecondSquared = 1;
        
            // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
            public static final double kRamseteB = 2;
            public static final double kRamseteZeta = 0.7;
        }

}
