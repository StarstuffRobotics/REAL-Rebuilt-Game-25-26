package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {
    public static final class Swerve {
        // --- CAN IDs ---
        public static final int FL_DRIVE_ID = 1;
        public static final int FL_ANGLE_ID = 2;

        public static final int FR_DRIVE_ID = 3;
        public static final int FR_ANGLE_ID = 4;

        public static final int BL_DRIVE_ID = 5;
        public static final int BL_ANGLE_ID = 6;

        public static final int BR_DRIVE_ID = 7;
        public static final int BR_ANGLE_ID = 8;

        // --- Physical Dimensions ---
        // Measurements for a 27.5" x 27.5" frame
        public static final double TRACK_WIDTH = Units.inchesToMeters(27.5); 
        public static final double WHEEL_BASE  = Units.inchesToMeters(27.5); 

        public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE / 2,  TRACK_WIDTH / 2),  // Front Left
            new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),  // Front Right
            new Translation2d(-WHEEL_BASE / 2,  TRACK_WIDTH / 2), // Back Left
            new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2)  // Back Right
        );

        // --- Performance Config ---
        public static final double MAX_SPEED = 4.5; // meters per second
        public static final double MAX_ANGULAR_SPEED = Math.PI * 2; // radians per second

        // --- Steering PID (Tune if wheels oscillate) ---
        public static final double angleP = 0.5;
        public static final double angleI = 0.0;
        public static final double angleD = 0.0;
    }

    public static final class OI {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final double DEADBAND = 0.1;
    }
}