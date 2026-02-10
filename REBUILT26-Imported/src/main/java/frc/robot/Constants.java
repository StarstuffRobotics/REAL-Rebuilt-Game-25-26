package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {
    public static final class DriveConstants {
        // --- CAN IDs (Update these) ---
        public static final int kFrontLeftDriveId = 11;
        public static final int kFrontLeftSteerId = 15;
        public static final int kFrontLeftEncoderId = 12;////////ask

        public static final int kFrontRightDriveId = 8;
        public static final int kFrontRightSteerId = 9;
        public static final int kFrontRightEncoderId = 10;

        public static final int kBackLeftDriveId = 2;
        public static final int kBackLeftSteerId = 3;
        public static final int kBackLeftEncoderId = 4;

        public static final int kBackRightDriveId = 5;
        public static final int kBackRightSteerId = 6;
        public static final int kBackRightEncoderId = 7;

        // --- Offsets in Rotations (0 to 1) ---
        public static final double kFrontLeftOffset = 0.0;
        public static final double kFrontRightOffset = 0.0;
        public static final double kBackLeftOffset = 0.0;
        public static final double kBackRightOffset = 0.0;

        // --- MK4i L2 Physical Constants ---
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4.0);
        public static final double kDriveGearRatio = 6.75;
        public static final double kSteerGearRatio = 150.0 / 7.0;

        public static final double kDrivePositionConversion = (Math.PI * kWheelDiameterMeters) / kDriveGearRatio;
        public static final double kDriveVelocityConversion = kDrivePositionConversion / 60.0;
        public static final double kSteerPositionConversion = (2 * Math.PI) / kSteerGearRatio;

        // Kinematics (Assuming a 24x24 inch robot frame)
        public static final double kTrackWidth = Units.inchesToMeters(24);
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kTrackWidth / 2, kTrackWidth / 2),
            new Translation2d(kTrackWidth / 2, -kTrackWidth / 2),
            new Translation2d(-kTrackWidth / 2, kTrackWidth / 2),
            new Translation2d(-kTrackWidth / 2, -kTrackWidth / 2)
        );
    }
}
