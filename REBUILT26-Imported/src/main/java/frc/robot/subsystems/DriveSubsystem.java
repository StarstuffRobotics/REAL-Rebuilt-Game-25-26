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
import frc.robot.Configs.MAXSwerveModule; // Ensure this is the correct package for MAXSwerveModule
import frc.robot.Configs.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
    // Mirroring logic: Left and Right sides often need opposite steering inversions
    private final SwerveModule m_frontLeft = new SwerveModule(
        DriveConstants.kFrontLeftDrivingCanId,
        DriveConstants.kFrontLeftTurningCanId,
        DriveConstants.kFrontLeftChassisAngularOffset
    );

    private final SwerveModule m_frontRight = new SwerveModule(
        DriveConstants.kFrontRightDrivingCanId,
        DriveConstants.kFrontRightTurningCanId,
        DriveConstants.kFrontRightChassisAngularOffset
    );

    private final SwerveModule m_rearLeft = new SwerveModule(
        DriveConstants.kRearLeftDrivingCanId,
        DriveConstants.kRearLeftTurningCanId,
        DriveConstants.kBackLeftChassisAngularOffset
    );

    private final SwerveModule m_rearRight = new SwerveModule(
        DriveConstants.kRearRightDrivingCanId,
        DriveConstants.kRearRightTurningCanId,
        DriveConstants.kBackRightChassisAngularOffset
    );

    private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();
    
    private final AHRS navx = new AHRS(AHRS.NavXComType.kMXP_SPI);
    
    private SwerveModuleState[] swerveModuleStates;

    // Define the swerve drive kinematics
    private final SwerveDriveKinematics m_kinematics = Constants.Swerve.KINEMATICS;

    // Define the swerve drive odometry
    private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
        m_kinematics,
        Rotation2d.fromDegrees(m_gyro.getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
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
        
        
        m_frontLeft.setState(states[0]);
        m_frontRight.setState(states[1]);
        m_rearLeft.setState(states[2]);
        m_rearRight.setState(states[3]);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Gyro Heading", getHeading().getDegrees());
        // Track the relative vs absolute position to see if they are drifting
        SmartDashboard.putNumber("Swerve/FR/Abs Angle", m_frontRight.getAbsolutePosition());
        SmartDashboard.putNumber("Swerve/FR/Rel Angle", m_frontRight.getRelativePosition());
        
        m_odometry.update(
        Rotation2d.fromDegrees(m_gyro.getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
    }
    
    

    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(Constants.Swerve.INVERT_GYRO ? -navx.getAngle() : navx.getAngle());
    }
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
            desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
        m_frontLeft.setState(desiredStates[0]);
        m_frontRight.setState(desiredStates[1]);
        m_rearLeft.setState(desiredStates[2]);
        m_rearRight.setState(desiredStates[3]);
    }
    
    
    public void zeroHeading() { navx.reset(); 
    }
    
    public SwerveModuleState[] getSwerveModuleStates() {
        return new SwerveModuleState[] {
            m_frontLeft.getState(),
            m_frontRight.getState(),
            m_rearLeft.getState(),
            m_rearRight.getState()
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
            Rotation2d.fromDegrees(m_gyro.getAngle())
        );
    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }
}