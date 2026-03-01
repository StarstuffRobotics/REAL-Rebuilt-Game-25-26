package frc.robot.subsystems;

// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
// import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Swerve;
import frc.robot.SwerveModule;

public class DriveSubsystem extends SubsystemBase {

    // Only the front right module for testing
    private final SwerveModule m_frontRight = new SwerveModule(
        Swerve.FR_DRIVE_ID, Swerve.FR_ANGLE_ID, Swerve.FR_CANCODER_ID,
        Swerve.FR_OFFSET, Swerve.FR_INVERTED);

    public DriveSubsystem() {}

    public void drive(double speed) {
        m_frontRight.setDriveSpeed(speed);
    }

    public void stop() {
        m_frontRight.setDriveSpeed(0);
    }
}
