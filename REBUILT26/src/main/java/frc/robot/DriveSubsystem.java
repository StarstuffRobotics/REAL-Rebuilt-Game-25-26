package frc.robot;

// DriveSubsystem.java
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {

  private final SwerveModule frontLeft  = new SwerveModule(1, 2);
  private final SwerveModule frontRight = new SwerveModule(3, 4);
  private final SwerveModule backLeft   = new SwerveModule(5, 6);
  private final SwerveModule backRight  = new SwerveModule(7, 8);

  public void drive(double xSpeed, double ySpeed, double rot) {
    // Robot-centric → NO gyro correction
    ChassisSpeeds speeds =
        new ChassisSpeeds(
            xSpeed * Constants.Swerve.MAX_SPEED,
            ySpeed * Constants.Swerve.MAX_SPEED,
            rot * Constants.Swerve.MAX_ANGULAR_SPEED
        );

    SwerveModuleState[] states =
        Constants.Swerve.KINEMATICS.toSwerveModuleStates(speeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(
        states, Constants.Swerve.MAX_SPEED
    );

    frontLeft.setState(states[0]);
    frontRight.setState(states[1]);
    backLeft.setState(states[2]);
    backRight.setState(states[3]);
  }
}
