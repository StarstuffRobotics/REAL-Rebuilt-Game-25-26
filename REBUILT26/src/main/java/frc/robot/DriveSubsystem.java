package frc.robot;

import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
    private final SwerveModule frontLeft = new SwerveModule(Constants.Swerve.FL_DRIVE_ID, Constants.Swerve.FL_ANGLE_ID);
    private final SwerveModule frontRight = new SwerveModule(Constants.Swerve.FR_DRIVE_ID, Constants.Swerve.FR_ANGLE_ID);
    private final SwerveModule backLeft = new SwerveModule(Constants.Swerve.BL_DRIVE_ID, Constants.Swerve.BL_ANGLE_ID);
    private final SwerveModule backRight = new SwerveModule(Constants.Swerve.BR_DRIVE_ID, Constants.Swerve.BR_ANGLE_ID);

    // If you add a gyro later, you would use it in the fromFieldRelativeSpeeds method
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        ChassisSpeeds speeds;
        
        if (fieldRelative) {
            // Placeholder: currently assumes 0 degrees since we aren't using AHRS
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, new edu.wpi.first.math.geometry.Rotation2d(0));
        } else {
            speeds = new ChassisSpeeds(xSpeed, ySpeed, rot);
        }

        SwerveModuleState[] states = Constants.Swerve.KINEMATICS.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Swerve.MAX_SPEED);

        frontLeft.setState(states[0]);
        frontRight.setState(states[1]);
        backLeft.setState(states[2]);
        backRight.setState(states[3]);
    }

    // Dummy method so RobotContainer doesn't crash, since we removed AHRS
    public void zeroHeading() {}
}