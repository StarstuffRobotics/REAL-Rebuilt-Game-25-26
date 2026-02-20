package frc.robot.subsystems;

import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveModule;
import frc.robot.Constants.Swerve;
import frc.robot.*;
//import frc.robot.Configs.MAXSwerveModule; // Ensure this is the correct package for MAXSwerveModule
import frc.robot.Configs.DriveConstants;
import com.studica.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@SuppressWarnings("unused")
public class DriveSubsystem extends SubsystemBase {
    // Mirroring logic: Left and Right sides often need opposite steering inversions
    
    private final SwerveModule frontLeft = new SwerveModule(
        Constants.Swerve.FL_DRIVE_ID, Constants.Swerve.FL_ANGLE_ID, Constants.Swerve.FL_CANCODER_ID, Constants.Swerve.FL_OFFSET, true);

    private final SwerveModule frontRight = new SwerveModule(
        Constants.Swerve.FR_DRIVE_ID, Constants.Swerve.FR_ANGLE_ID, Constants.Swerve.FR_CANCODER_ID, Constants.Swerve.FR_OFFSET, false);

    private final SwerveModule backLeft = new SwerveModule(
        Constants.Swerve.BL_DRIVE_ID, Constants.Swerve.BL_ANGLE_ID, Constants.Swerve.BL_CANCODER_ID, Constants.Swerve.BL_OFFSET, true);

    private final SwerveModule backRight = new SwerveModule(
        Constants.Swerve.BR_DRIVE_ID, Constants.Swerve.BR_ANGLE_ID, Constants.Swerve.BR_CANCODER_ID, Constants.Swerve.BR_OFFSET, false);

        
    private final AHRS navx = new AHRS(AHRS.NavXComType.kMXP_SPI);
    
    private SwerveModuleState[] swerveModuleStates;

    // Define the swerve drive kinematics
    private final SwerveDriveKinematics m_kinematics = Constants.Swerve.KINEMATICS;

    // Define the swerve drive odometry
    private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
        m_kinematics,
        Rotation2d.fromDegrees(navx.getAngle()),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        }
    );

    public DriveSubsystem() {
        new Thread(() -> {
            try { Thread.sleep(1000); zeroHeading(); } catch (Exception e) {}
        }).start();
    }

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        ChassisSpeeds speeds = fieldRelative 
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getHeading()) 
            : new ChassisSpeeds(xSpeed, ySpeed, rot);

        SwerveModuleState[] states = Constants.Swerve.KINEMATICS.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Swerve.MAX_SPEED);
        
        
        frontLeft.setState(states[0]);
        frontRight.setState(states[1]);
        backLeft.setState(states[2]);
        backRight.setState(states[3]);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Gyro Heading", getHeading().getDegrees());
        // Track the relative vs absolute position to see if they are drifting
        SmartDashboard.putNumber("Swerve/FR/Abs Angle", frontRight.getAbsolutePosition());
        SmartDashboard.putNumber("Swerve/FR/Rel Angle", frontRight.getRelativePosition());
        
        m_odometry.update(
        Rotation2d.fromDegrees(navx.getAngle()),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        });
    }
    
    

    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(Constants.Swerve.INVERT_GYRO ? -navx.getAngle() : navx.getAngle());
    }
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
            desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
        frontLeft.setState(desiredStates[0]);
        frontRight.setState(desiredStates[1]);
        backLeft.setState(desiredStates[2]);
        backRight.setState(desiredStates[3]);
    }
    
    
    public void zeroHeading() { navx.reset(); 
    }
    
    public SwerveModuleState[] getSwerveModuleStates() {
        return new SwerveModuleState[] {
            frontLeft.getState(),
            frontRight.getState(),
            backLeft.getState(),
            backRight.getState()
        };
    }


    public ChassisSpeeds getFieldSpeeds() {
        // Get the current states of the swerve modules
        SwerveModuleState[] currentStates = getSwerveModuleStates();
    
        // Convert the module states to chassis speeds
        ChassisSpeeds robotSpeeds = m_kinematics.toChassisSpeeds(currentStates);
    
        // Convert robot-relative speeds to field-relative speeds using the gyro angle
        return ChassisSpeeds.fromFieldRelativeSpeeds(
            robotSpeeds.vxMetersPerSecond,
            robotSpeeds.vyMetersPerSecond,
            robotSpeeds.omegaRadiansPerSecond,
            Rotation2d.fromDegrees(navx.getAngle())
        );
    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }
}