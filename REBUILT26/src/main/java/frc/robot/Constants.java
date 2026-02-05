package frc.robot;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public final class Constants {

  public static final class Swerve {
    public static final double TRACK_WIDTH = 0.5842; // meters (23 in)
    public static final double WHEEL_BASE  = 0.5842; // meters

    public static final double MAX_SPEED = 4.5; // m/s
    public static final double MAX_ANGULAR_SPEED = Math.PI * 2; // rad/s

    public static final SwerveDriveKinematics KINEMATICS =
        new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE / 2,  TRACK_WIDTH / 2),  // FL
            new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),  // FR
            new Translation2d(-WHEEL_BASE / 2,  TRACK_WIDTH / 2), // BL
            new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2)  // BR
        );
  }
}
