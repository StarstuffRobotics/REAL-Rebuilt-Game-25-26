package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
    // Initialize your four modules using the IDs from Constants
    private final SwerveModule frontLeft = new SwerveModule(Constants.Swerve.FL_DRIVE_ID, Constants.Swerve.FL_ANGLE_ID, Constants.Swerve.FL_CANCODER_ID, Constants.Swerve.FL_OFFSET, Constants.Swerve.FL_INVERTED);
    private final SwerveModule frontRight = new SwerveModule(Constants.Swerve.FR_DRIVE_ID, Constants.Swerve.FR_ANGLE_ID, Constants.Swerve.FR_CANCODER_ID, Constants.Swerve.FR_OFFSET, Constants.Swerve.FR_INVERTED);
    private final SwerveModule backLeft = new SwerveModule(Constants.Swerve.BL_DRIVE_ID, Constants.Swerve.BL_ANGLE_ID, Constants.Swerve.BL_CANCODER_ID, Constants.Swerve.BL_OFFSET, Constants.Swerve.BL_INVERTED);
    private final SwerveModule backRight = new SwerveModule(Constants.Swerve.BR_DRIVE_ID, Constants.Swerve.BR_ANGLE_ID, Constants.Swerve.BR_CANCODER_ID, Constants.Swerve.BR_OFFSET, Constants.Swerve.BR_INVERTED);

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        ChassisSpeeds speeds;
        
        if (fieldRelative) {
            // Using a new Rotation2d(0) as a placeholder since no gyro is connected yet
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, new Rotation2d(0));
        } else {
            speeds = new ChassisSpeeds(xSpeed, ySpeed, rot);
        }

        // Convert chassis speeds to individual module states
        SwerveModuleState[] states = Constants.Swerve.KINEMATICS.toSwerveModuleStates(speeds);
        
        // Ensure the robot doesn't try to exceed its maximum physical speed
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Swerve.MAX_SPEED);

        // Apply the calculated states to each physical module
        frontLeft.setState(states[0]);
        frontRight.setState(states[1]);
        backLeft.setState(states[2]);
        backRight.setState(states[3]);
    }

    // Placeholder for gyro resetting
    public void zeroHeading() {}
}