package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.io.File;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

public class SwerveSubsystem extends SubsystemBase {
  private final SwerveDrive m_swerveDrive;

  public SwerveSubsystem() {
    try {
      File swerveConfigDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
      m_swerveDrive =
          new SwerveParser(swerveConfigDirectory)
              .createSwerveDrive(Constants.SwerveConstants.kMaxSpeedMetersPerSecond);
    } catch (Exception e) {
      throw new RuntimeException("Unable to load YAGSL swerve configuration.", e);
    }

    m_swerveDrive.setHeadingCorrection(false);
  }

  public void drive(double xInput, double yInput, double rotInput, boolean fieldRelative) {
    Translation2d translation =
        new Translation2d(
            xInput * m_swerveDrive.getMaximumChassisVelocity(),
            yInput * m_swerveDrive.getMaximumChassisVelocity());
    double rotation = rotInput * m_swerveDrive.getMaximumChassisAngularVelocity();
    m_swerveDrive.drive(translation, rotation, fieldRelative, false);
  }

  public void stop() {
    m_swerveDrive.drive(new Translation2d(), 0.0, true, false);
  }

  public void zeroGyro() {
    m_swerveDrive.zeroGyro();
  }

  public void lockModules() {
    m_swerveDrive.lockPose();
  }
}
