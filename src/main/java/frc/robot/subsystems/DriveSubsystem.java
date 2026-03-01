package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Swerve;
import frc.robot.SwerveModule;

public class DriveSubsystem extends SubsystemBase {

    // Only the front right module for testing
    private final SwerveModule m_frontRight = new SwerveModule(
        Swerve.FR_DRIVE_ID, Swerve.FR_ANGLE_ID, Swerve.FR_CANCODER_ID,
        Swerve.FR_OFFSET, Swerve.FR_INVERTED);

    public DriveSubsystem() {}

    /**
     * Drive using joystick inputs (-1 to 1). Robot-centric only.
     * Computes kinematics for all 4 modules but only commands front right.
     */
    public void drive(double xSpeed, double ySpeed, double rot) {
        ChassisSpeeds speeds = new ChassisSpeeds(
            xSpeed * Swerve.MAX_SPEED,
            ySpeed * Swerve.MAX_SPEED,
            rot * Swerve.MAX_ANGULAR_SPEED
        );

        // Kinematics output order: FL[0], FR[1], BL[2], BR[3]
        SwerveModuleState[] states = Swerve.KINEMATICS.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Swerve.MAX_SPEED);

        // Only command front right
        m_frontRight.setState(states[1]);

        // Debug output
        SmartDashboard.putNumber("FR Speed", states[1].speedMetersPerSecond);
        SmartDashboard.putNumber("FR Angle", states[1].angle.getDegrees());
        SmartDashboard.putNumber("FR Actual Angle", m_frontRight.getAngle().getDegrees());
    }

    public void stop() {
        m_frontRight.stop();
    }

    public void setX() {
        m_frontRight.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    }
}
