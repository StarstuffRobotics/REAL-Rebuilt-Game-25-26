package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {
    public static final class DriveConstants {
        // --- CAN IDs ---
        public static final int kFrontLeftDriveId = 10;
        public static final int kFrontLeftSteerId = 14;
        public static final int kFrontLeftEncoderId = 11;
        public static final int kFrontRightDriveId = 7;
        public static final int kFrontRightSteerId = 8;
        public static final int kFrontRightEncoderId = 12;
        public static final int kBackLeftDriveId = 1;
        public static final int kBackLeftSteerId = 2;
        public static final int kBackLeftEncoderId = 13;
        public static final int kBackRightDriveId = 4;
        public static final int kBackRightSteerId = 5;
        public static final int kBackRightEncoderId = 14;

        // Kinematics (24x24 inch frame)
        public static final double kTrackWidth = Units.inchesToMeters(24);
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kTrackWidth / 2, kTrackWidth / 2),
            new Translation2d(kTrackWidth / 2, -kTrackWidth / 2),
            new Translation2d(-kTrackWidth / 2, kTrackWidth / 2),
            new Translation2d(-kTrackWidth / 2, -kTrackWidth / 2)
        );
    }

    public static final class SpindexerConstants {
        public static final int kSpindexerMotorId = 22;
        public static final double kSpindexerSpeed = 0.5;
    }
//not doing intake code because it is already done
    // public static final class IntakeConstants {
    //     public static final int kPivotMotorId = 20;
    //     public static final int kRollerMotorId = 21;
    //     public static final double kRollerSpeed = 0.7;
    //     public static final double kPivotPositionDown = 0.0;
    //     public static final double kPivotPositionUp = 15.0;
    //     public static final double kPositionTolerance = 0.5;
    // }
public static final class AcceleratorConstants {
    public static final int kAcceleratorMotorId = 23;
    public static final double kAcceleratorSpeed = 0.8; // Accelerators speed we are gonna chnage as neccesary
}
}