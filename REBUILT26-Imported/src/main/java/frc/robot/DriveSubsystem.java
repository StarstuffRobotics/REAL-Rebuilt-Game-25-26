package frc.robot;

import com.studica.frc.AHRS; // Updated for 2026 Studica Library
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
    // --- Swerve Modules ---
    private final SwerveModule frontLeft = new SwerveModule(
        Constants.Swerve.FL_DRIVE_ID, 
        Constants.Swerve.FL_ANGLE_ID, 
        Constants.Swerve.FL_CANCODER_ID, 
        Constants.Swerve.FL_OFFSET);

    private final SwerveModule frontRight = new SwerveModule(
        Constants.Swerve.FR_DRIVE_ID, 
        Constants.Swerve.FR_ANGLE_ID, 
        Constants.Swerve.FR_CANCODER_ID, 
        Constants.Swerve.FR_OFFSET);

    private final SwerveModule backLeft = new SwerveModule(
        Constants.Swerve.BL_DRIVE_ID, 
        Constants.Swerve.BL_ANGLE_ID, 
        Constants.Swerve.BL_CANCODER_ID, 
        Constants.Swerve.BL_OFFSET);

    private final SwerveModule backRight = new SwerveModule(
        Constants.Swerve.BR_DRIVE_ID, 
        Constants.Swerve.BR_ANGLE_ID, 
        Constants.Swerve.BR_CANCODER_ID, 
        Constants.Swerve.BR_OFFSET);

    // --- navX2 Gyro ---
    // In 2026, we specify NavXComType.kMXP_SPI for the MXP port connection
    private final AHRS navx = new AHRS(AHRS.NavXComType.kMXP_SPI);

    public DriveSubsystem() {
        // We wait a second and then zero the gyro to ensure it's calibrated
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {}
        }).start();
    }

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        ChassisSpeeds speeds;
        
        if (fieldRelative) {
            // Now using the real gyro heading instead of new Rotation2d(0)
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getHeading());
        } else {
            speeds = new ChassisSpeeds(xSpeed, ySpeed, rot);
        }

        // Convert speeds to module states
        SwerveModuleState[] states = Constants.Swerve.KINEMATICS.toSwerveModuleStates(speeds);
        
        // Prevent motors from trying to exceed physical limits
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Swerve.MAX_SPEED);

        // Apply states to modules
        frontLeft.setState(states[0]);
        frontRight.setState(states[1]);
        backLeft.setState(states[2]);
        backRight.setState(states[3]);
    }

    @Override
    public void periodic() {
        // Good for debugging! Check this on the dashboard to see if the robot knows its angle
        SmartDashboard.putNumber("Gyro Heading", getHeading().getDegrees());
    }

    /**
     * Returns the heading of the robot.
     * WPILib expects Counter-Clockwise (CCW) to be positive.
     */
    public Rotation2d getHeading() {
        // navX is CW positive, so we negate it based on the constant setting
        return Rotation2d.fromDegrees(Constants.Swerve.INVERT_GYRO ? -navx.getAngle() : navx.getAngle());
    }

    public void zeroHeading() {
        navx.reset();
    }
}