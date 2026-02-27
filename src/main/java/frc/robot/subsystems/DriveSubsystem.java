package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import com.studica.frc.AHRS;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Swerve;
import frc.robot.SwerveModule;

public class DriveSubsystem extends SubsystemBase {

    private final SwerveModule m_frontLeft = new SwerveModule(
        Swerve.FL_DRIVE_ID, Swerve.FL_ANGLE_ID, Swerve.FL_CANCODER_ID,
        Swerve.FL_OFFSET, Swerve.FL_INVERTED);

    private final SwerveModule m_frontRight = new SwerveModule(
        Swerve.FR_DRIVE_ID, Swerve.FR_ANGLE_ID, Swerve.FR_CANCODER_ID,
        Swerve.FR_OFFSET, Swerve.FR_INVERTED);

    private final SwerveModule m_rearLeft = new SwerveModule(
        Swerve.BL_DRIVE_ID, Swerve.BL_ANGLE_ID, Swerve.BL_CANCODER_ID,
        Swerve.BL_OFFSET, Swerve.BL_INVERTED);

    private final SwerveModule m_rearRight = new SwerveModule(
        Swerve.BR_DRIVE_ID, Swerve.BR_ANGLE_ID, Swerve.BR_CANCODER_ID,
        Swerve.BR_OFFSET, Swerve.BR_INVERTED);

    private final AHRS m_navx = new AHRS(AHRS.NavXComType.kMXP_SPI);
    private final Field2d m_field = new Field2d();

    // Slew rate limiters for smoother acceleration (units/sec rate of change)
    private final SlewRateLimiter m_xLimiter = new SlewRateLimiter(3.0);
    private final SlewRateLimiter m_yLimiter = new SlewRateLimiter(3.0);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3.0);

    private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
        Swerve.KINEMATICS,
        getHeading(),
        getModulePositions()
    );

    public DriveSubsystem() {
        new Thread(() -> {
            try { Thread.sleep(1000); zeroHeading(); } catch (Exception e) {}
        }).start();
        SmartDashboard.putData("Field", m_field);

        // PathPlanner AutoBuilder configuration
        RobotConfig config = null;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
        }

        AutoBuilder.configure(
            this::getPose,
            this::resetPose,
            this::getRobotRelativeSpeeds,
            (speeds, feedforwards) -> driveRobotRelative(speeds),
            new PPHolonomicDriveController(
                new PIDConstants(5.0, 0.0, 0.0), // Translation PID
                new PIDConstants(5.0, 0.0, 0.0)  // Rotation PID
            ),
            config,
            () -> {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this
        );
    }

    /**
     * Drive the robot using joystick inputs (-1 to 1).
     * Applies slew rate limiting and scales to max speed.
     */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        // Apply slew rate limiting for smoother acceleration
        double limitedX = m_xLimiter.calculate(xSpeed);
        double limitedY = m_yLimiter.calculate(ySpeed);
        double limitedRot = m_rotLimiter.calculate(rot);

        // Scale joystick (-1 to 1) to actual speeds (m/s and rad/s)
        double xSpeedDelivered = limitedX * Swerve.MAX_SPEED;
        double ySpeedDelivered = limitedY * Swerve.MAX_SPEED;
        double rotDelivered = limitedRot * Swerve.MAX_ANGULAR_SPEED;

        ChassisSpeeds speeds = fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, getHeading())
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered);

        SwerveModuleState[] states = Swerve.KINEMATICS.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Swerve.MAX_SPEED);

        m_frontLeft.setState(states[0]);
        m_frontRight.setState(states[1]);
        m_rearLeft.setState(states[2]);
        m_rearRight.setState(states[3]);
    }

    /**
     * Drive the robot given robot-relative ChassisSpeeds (in m/s and rad/s).
     * Used by PathPlanner — does NOT apply slew rate limiting or joystick scaling.
     */
    public void driveRobotRelative(ChassisSpeeds speeds) {
        SwerveModuleState[] states = Swerve.KINEMATICS.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Swerve.MAX_SPEED);
        m_frontLeft.setState(states[0]);
        m_frontRight.setState(states[1]);
        m_rearLeft.setState(states[2]);
        m_rearRight.setState(states[3]);
    }

    @Override
    public void periodic() {
        m_odometry.update(getHeading(), getModulePositions());
        m_field.setRobotPose(m_odometry.getPoseMeters());
        SmartDashboard.putNumber("Gyro Heading", getHeading().getDegrees());
    }

    public Rotation2d getHeading() {
        // We use getPitch() because the gyro is mounted vertically
        return Rotation2d.fromDegrees(Swerve.INVERT_GYRO ? m_navx.getPitch() : -m_navx.getPitch());
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            m_frontLeft.getPosition(), m_frontRight.getPosition(),
            m_rearLeft.getPosition(), m_rearRight.getPosition()
        };
    }

    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            m_frontLeft.getState(), m_frontRight.getState(),
            m_rearLeft.getState(), m_rearRight.getState()
        };
    }

    public void zeroHeading() { m_navx.reset(); }

    public Pose2d getPose() { return m_odometry.getPoseMeters(); }

    public void resetPose(Pose2d pose) {
        m_odometry.resetPosition(getHeading(), getModulePositions(), pose);
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return Swerve.KINEMATICS.toChassisSpeeds(getModuleStates());
    }

    public ChassisSpeeds getFieldSpeeds() {
        return ChassisSpeeds.fromFieldRelativeSpeeds(getRobotRelativeSpeeds(), getHeading());
    }

    public double getTurnRate() {
        return m_navx.getRate() * (Swerve.INVERT_GYRO ? -1.0 : 1.0);
    }

    public void stopModules() {
        m_frontLeft.stop();
        m_frontRight.stop();
        m_rearLeft.stop();
        m_rearRight.stop();
    }

    public void setX() {
        m_frontLeft.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        m_frontRight.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_rearLeft.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_rearRight.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    public void resetEncoders() {
        m_frontLeft.resetToAbsolute();
        m_frontRight.resetToAbsolute();
        m_rearLeft.resetToAbsolute();
        m_rearRight.resetToAbsolute();
    }
}
