package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {

    public static final class Swerve {
        // --- CAN IDs: Drive Motors ---
        public static final int FL_DRIVE_ID = 11;
        public static final int FR_DRIVE_ID = 8;
        public static final int BL_DRIVE_ID = 2;
        public static final int BR_DRIVE_ID = 5;

        // --- CAN IDs: Steering Motors ---
        public static final int FL_ANGLE_ID = 15;
        public static final int FR_ANGLE_ID = 9;
        public static final int BL_ANGLE_ID = 3;
        public static final int BR_ANGLE_ID = 6;

        // --- CAN IDs: CANcoders ---
        public static final int FL_CANCODER_ID = 12;
        public static final int FR_CANCODER_ID = 10;
        public static final int BL_CANCODER_ID = 4;
        public static final int BR_CANCODER_ID = 7;

        // --- CANcoder Offsets (Rotations) ---
        // Align wheels perfectly straight, read "Absolute Position" in Tuner X, and enter here.
        public static final double FL_OFFSET = 0.0; 
        public static final double FR_OFFSET = 0.0;
        public static final double BL_OFFSET = 0.0;
        public static final double BR_OFFSET = 0.0;

        // --- Physical Dimensions (27.5" x 27.5" frame) ---
        public static final double TRACK_WIDTH = Units.inchesToMeters(27.5); 
        public static final double WHEEL_BASE  = Units.inchesToMeters(27.5); 

        public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE / 2,  TRACK_WIDTH / 2),  // Front Left
            new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),  // Front Right
            new Translation2d(-WHEEL_BASE / 2,  TRACK_WIDTH / 2), // Back Left
            new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2)  // Back Right
        );

        // --- Drive Settings ---
        public static final double MAX_SPEED = 4.5; // m/s
        public static final double MAX_ANGULAR_SPEED = Math.PI * 2; // rad/s

        // --- Steering PID ---
        public static final double angleP = 0.5;
        public static final double angleI = 0.0;
        public static final double angleD = 0.0;
    }

    public static final class OI {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final double DEADBAND = 0.1;
    }
}