package frc.robot.subsystems;
//
import com.studica.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Swerve;
import frc.robot.SwerveModule;

public class DriveSubsystem extends SubsystemBase {
    // Standardizing inversions: Most Swerve setups use the same orientation for all 4.
    // If the robot drives backward, flip the drive motor inversion in SwerveModule.
    private final SwerveModule m_frontLeft = new SwerveModule(
        Swerve.FL_DRIVE_ID, 
        Swerve.FL_ANGLE_ID, 
        Swerve.FL_CANCODER_ID, 
        Swerve.FL_OFFSET, 
        Swerve.FL_INVERTED);

    private final SwerveModule m_frontRight = new SwerveModule(
        Swerve.FR_DRIVE_ID, 
        Swerve.FR_ANGLE_ID, 
        Swerve.FR_CANCODER_ID, 
        Swerve.FR_OFFSET, 
        Swerve.FR_INVERTED);

    private final SwerveModule m_rearLeft = new SwerveModule(
        Swerve.BL_DRIVE_ID, 
        Swerve.BL_ANGLE_ID, 
        Swerve.BL_CANCODER_ID, 
        Swerve.BL_OFFSET, 
        Swerve.BL_INVERTED);

    private final SwerveModule m_rearRight = new SwerveModule(
        Swerve.BR_DRIVE_ID, 
        Swerve.BR_ANGLE_ID, 
        Swerve.BR_CANCODER_ID, 
        Swerve.BR_OFFSET, 
        Swerve.BR_INVERTED);

    private final AHRS m_navx = new AHRS(AHRS.NavXComType.kMXP_SPI);
    private final Field2d m_field = new Field2d();

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
    }

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        // WPILib NWU coordinates: x is forward, y is left.
        ChassisSpeeds speeds = fieldRelative 
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getHeading()) 
            : new ChassisSpeeds(xSpeed, ySpeed, rot);

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
    // We use getPitch() because the gyro is vertical
        return Rotation2d.fromDegrees(Swerve.INVERT_GYRO ? m_navx.getPitch() : -m_navx.getPitch());
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            m_frontLeft.getPosition(), m_frontRight.getPosition(),
            m_rearLeft.getPosition(), m_rearRight.getPosition()
        };
    }

    public void zeroHeading() { m_navx.reset(); }

    public Pose2d getPose() { return m_odometry.getPoseMeters(); }

    public ChassisSpeeds getFieldSpeeds() {
        return ChassisSpeeds.fromFieldRelativeSpeeds(
            Swerve.KINEMATICS.toChassisSpeeds(getModuleStates()), getHeading());
    }

    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            m_frontLeft.getState(), m_frontRight.getState(),
            m_rearLeft.getState(), m_rearRight.getState()
        };
    }

    public void setX() {
        m_frontLeft.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        m_frontRight.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_rearLeft.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_rearRight.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

}