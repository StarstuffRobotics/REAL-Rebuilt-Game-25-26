
package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
    private final SwerveModule m_frontLeft = new SwerveModule(
        DriveConstants.kFrontLeftDriveId, DriveConstants.kFrontLeftSteerId, 
        DriveConstants.kFrontLeftEncoderId, DriveConstants.kFrontLeftOffset);

    private final SwerveModule m_frontRight = new SwerveModule(
        DriveConstants.kFrontRightDriveId, DriveConstants.kFrontRightSteerId, 
        DriveConstants.kFrontRightEncoderId, DriveConstants.kFrontRightOffset);

    private final SwerveModule m_backLeft = new SwerveModule(
        DriveConstants.kBackLeftDriveId, DriveConstants.kBackLeftSteerId, 
        DriveConstants.kBackLeftEncoderId, DriveConstants.kBackLeftOffset);

    private final SwerveModule m_backRight = new SwerveModule(
        DriveConstants.kBackRightDriveId, DriveConstants.kBackRightSteerId, 
        DriveConstants.kBackRightEncoderId, DriveConstants.kBackRightOffset);

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        SwerveModuleState[] swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
            fieldRelative 
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getRotation())
                : new ChassisSpeeds(xSpeed, ySpeed, rot)
        );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, 4.5);
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_backLeft.setDesiredState(swerveModuleStates[2]);
        m_backRight.setDesiredState(swerveModuleStates[3]);
    }

    public Rotation2d getRotation() {
        // Typically you'd use a Gyro here, returning 0 for now
        return Rotation2d.fromDegrees(0);
    }
}
