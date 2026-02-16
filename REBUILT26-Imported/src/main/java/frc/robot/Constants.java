package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {
    public static final class Swerve {
        // --- CAN IDs ---
        public static final int FL_DRIVE_ID = 11;
        public static final int FL_ANGLE_ID = 15;
        public static final int FL_CANCODER_ID = 12;
        public static final double FL_OFFSET = 0.065; 
        
        public static final int FR_DRIVE_ID = 8;
        public static final int FR_ANGLE_ID = 9;
        public static final int FR_CANCODER_ID = 10;
        public static final double FR_OFFSET = 0.110;

        public static final int BL_DRIVE_ID = 2;
        public static final int BL_ANGLE_ID = 3;
        public static final int BL_CANCODER_ID = 4;
        public static final double BL_OFFSET = 0.0470; // Fixed typo from 0.0470
        
        public static final int BR_DRIVE_ID = 5;
        public static final int BR_ANGLE_ID = 6;
        public static final int BR_CANCODER_ID = 7;
        public static final double BR_OFFSET = -0.051;

        // --- Physical Constants ---
        public static final double WHEEL_DIAMETER = Units.inchesToMeters(3.0);
        public static final double STEER_GEAR_RATIO = 9.42; 
        public static final double DRIVE_GEAR_RATIO = 4.71; 

        public static final double DRIVE_ROTATIONS_TO_METERS = (WHEEL_DIAMETER * Math.PI) / DRIVE_GEAR_RATIO;
        public static final double STEER_ROTATIONS_TO_RADIANS = (2 * Math.PI) / STEER_GEAR_RATIO;

        public static final double TRACK_WIDTH = Units.inchesToMeters(27.5); 
        public static final double WHEEL_BASE  = Units.inchesToMeters(27.5); 

        // Kinematics order: FL, FR, BL, BR
        public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE / 2,  TRACK_WIDTH / 2),
            new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
            new Translation2d(-WHEEL_BASE / 2,  TRACK_WIDTH / 2),
            new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2)
        );

        public static final double MAX_SPEED = 4.5; 
        public static final double MAX_ANGULAR_SPEED = Math.PI * 2;

        public static final double angleP = 0.4; // Slightly lower for stability
        public static final double angleI = 0.0;
        public static final double angleD = 0.01;
        
        public static final boolean INVERT_GYRO = false;
    }

    public static final class OI {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final double DEADBAND = 0.1;
    }
}