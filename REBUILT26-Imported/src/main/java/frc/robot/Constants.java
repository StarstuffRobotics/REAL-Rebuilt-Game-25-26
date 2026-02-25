package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {
    public static final class Swerve {
        // --- CAN IDs ---
        public static final int FL_DRIVE_ID = 8;
        public static final int FL_ANGLE_ID = 9;
        public static final double FL_OFFSET = -Math.PI / 2; 

        public static final int FR_DRIVE_ID = 4; 
        public static final int FR_ANGLE_ID = 5; 
        public static final double FR_OFFSET = 0;

        public static final int BL_DRIVE_ID = 11;
        public static final int BL_ANGLE_ID = 7; 
        public static final double BL_OFFSET = Math.PI;

        public static final int BR_DRIVE_ID = 12; 
        public static final int BR_ANGLE_ID = 3;
        public static final double BR_OFFSET = Math.PI / 2;

        public static final int PIGEON_ID = 1;

        // --- Conversion Factors (Calculated for MAXSwerve/SparkFlex) ---
        // Example: 4 inch wheel, 4.71:1 drive ratio, 21.43:1 steer ratio
        public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
        public static final double DRIVE_MOTOR_REDUCTION = 4.71;
        public static final double STEER_MOTOR_REDUCTION = 21.43;

        public static final double DRIVE_ROTATIONS_TO_METERS = (WHEEL_DIAMETER * Math.PI) / DRIVE_MOTOR_REDUCTION;
        public static final double STEER_ROTATIONS_TO_RADIANS = (2 * Math.PI) / STEER_MOTOR_REDUCTION;

        // --- PID Gains ---
        public static final double angleP = 1.0;
        public static final double angleI = 0.0;
        public static final double angleD = 0.0;

        // --- Chassis Specs ---
        public static final double TRACK_WIDTH = Units.inchesToMeters(22.5);
        public static final double WHEEL_BASE = Units.inchesToMeters(22.5);

        public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
            new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
            new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
            new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2)
        );

        public static final double MAX_SPEED = 4.8; 
        public static final double MAX_ANGULAR_SPEED = 2 * Math.PI;
        public static final double DEADBAND = 0.1;
    }
}