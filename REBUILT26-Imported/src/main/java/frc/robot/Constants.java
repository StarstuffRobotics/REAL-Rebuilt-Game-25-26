package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import com.revrobotics.spark.SparkMax; // Import NeoMotorConstants

public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(22.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(22.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs

    // public static final int kFrontLeftDrivingCanId = 8;
    // public static final int kRearLeftDrivingCanId = 11;
    // public static final int kFrontRightDrivingCanId = 4;
    // public static final int kRearRightDrivingCanId = 12;

    // public static final int kFrontLeftTurningCanId = 9;
    // public static final int kRearLeftTurningCanId = 7;
    // public static final int kFrontRightTurningCanId = 5;
    // public static final int kRearRightTurningCanId = 3;


    // public static final int kFrontLeftDrivingCanId = 11;
    // public static final int kRearLeftDrivingCanId = 13;
    // public static final int kFrontRightDrivingCanId = 15;
    // public static final int kRearRightDrivingCanId = 17;

    // public static final int kFrontLeftTurningCanId = 10;
    // public static final int kRearLeftTurningCanId = 12;
    // public static final int kFrontRightTurningCanId = 14;
    // public static final int kRearRightTurningCanId = 16;

    public static final boolean kGyroReversed = false;
  }  
  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }
  public static final class Swerve {
        // --- CAN IDs ---
        public static final int FL_DRIVE_ID = 8;
        public static final int FL_ANGLE_ID = 9;
        public static final int FL_CANCODER_ID = 10;
        public static final double FL_OFFSET = 0.065; 
        
        public static final int FR_DRIVE_ID = 5;
        public static final int FR_ANGLE_ID = 6;
        public static final int FR_CANCODER_ID = 7;
        public static final double FR_OFFSET = 0.110;

        public static final int BL_DRIVE_ID = 11;
        public static final int BL_ANGLE_ID = 15;
        public static final int BL_CANCODER_ID = 12;
        public static final double BL_OFFSET = 0.0470; // Fixed typo from 0.0470
        
        public static final int BR_DRIVE_ID = 2;
        public static final int BR_ANGLE_ID = 3;
        public static final int BR_CANCODER_ID = 4;
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


    public static final class ModuleConstants {
        // The MAXSwerve module can be configured with one of three pinion gears: 12T,
        // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
        // more teeth will result in a robot that drives faster).
        public static final int kDrivingMotorPinionTeeth = 14;
    
        // Calculations required for driving motor conversion factors and feed forward
        public static final double kDrivingMotorFreeSpeedRps = 5676.0 / 60; // 5676 RPM is the free speed of a NEO motor
        
        
        public static final double kWheelDiameterMeters = 0.0762;
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
        // teeth on the bevel pinion
        public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
        public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
            / kDrivingMotorReduction;
    }
    
    public static final class OI {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final double DEADBAND = 0.1;
    }

    public static final class Dimensions{
        public static final double FULL_WIDTH = Units.inchesToMeters(27.5);
        public static final double FULL_LENGTH = Units.inchesToMeters(27.5);
        public static final double BUMPER_HEIGHT = Units.inchesToMeters(10);
            // Utility methods for unit conversion
          public static class Inches {
            public static double of(double inches) {
                return Units.inchesToMeters(inches); // Converts inches to meters
            }
          }
      
          public static class Meters {
            public static double of(double meters) {
              return meters; // No conversion needed for meters
            }
          }
      
          // Example method for dividing dimensions
          public static double div(double value, double divisor) {
            return value / divisor;
          }
      
          // Example method for adding dimensions
          public static double plus(double value1, double value2) {
            return value1 + value2;
          }

          public static final class NeoMotorConstants {
            public static final double kFreeSpeedRpm = 5676;
          }
      }
}