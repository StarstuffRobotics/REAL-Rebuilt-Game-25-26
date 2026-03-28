package frc.robot;

import edu.wpi.first.math.util.Units;

public final class Constants {
  private Constants() {}

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.08;
  }

  public static final class SwerveConstants {
    private SwerveConstants() {}

    public static final double kRobotLengthInches = 27.5;
    public static final double kRobotWidthInches = 27.5;
    public static final double kHalfRobotLengthInches = kRobotLengthInches / 2.0;
    public static final double kHalfRobotWidthInches = kRobotWidthInches / 2.0;

    // MK4i L2 with NEO Vortex is typically around 4.7-5.0 m/s free speed on 12V.
    public static final double kMaxSpeedMetersPerSecond = Units.feetToMeters(15.5);
  }
}
