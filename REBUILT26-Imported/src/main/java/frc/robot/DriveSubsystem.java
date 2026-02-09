package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
    // Hardware: Four Modules
    // We now pass the ACTUAL constants instead of placeholders (0, 0)
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

    // Gyroscope (Pigeon 2.0 is standard for many swerve builds)
    // Change the ID if your gyro is different!
    private final Pigeon2 gyro = new Pigeon2(1);

    public DriveSubsystem() {
        // Optional: Reset gyro on startup
        zeroHeading();
    }

    @Override
    public void periodic() {
        // Log telemetry to SmartDashboard so you can see if the code is alive
        SmartDashboard.putNumber("Robot Heading", getHeading().getDegrees());
    }

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        ChassisSpeeds speeds;
        
        if (fieldRelative) {
            // Converts joystick inputs relative to the field using gyro feedback
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getHeading());
        } else {
            // Standard robot-centric driving
            speeds = new ChassisSpeeds(xSpeed, ySpeed, rot);
        }

        // Calculate module states
        SwerveModuleState[] states = Constants.Swerve.KINEMATICS.toSwerveModuleStates(speeds);
        
        // Prevents the robot from trying to drive faster than physically possible
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Swerve.MAX_SPEED);

        // Send states to modules
        frontLeft.setState(states[0]);
        frontRight.setState(states[1]);
        backLeft.setState(states[2]);
        backRight.setState(states[3]);
    }

    /** Returns the current rotation of the robot from the gyro */
    public Rotation2d getHeading() {
        return gyro.getRotation2d();
    }

    /** Resets the gyro to 0. Call this when the robot is facing away from the driver. */
    public void zeroHeading() {
        gyro.reset();
    }
}