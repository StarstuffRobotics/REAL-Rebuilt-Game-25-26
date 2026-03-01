// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package main.java.frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveDriveSubsystem extends SubsystemBase
{
	// WHEEL: 4 inches -> meters (for reference / future use)
	private static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4.0);

	// NOTE: Cancoder/steer CAN IDs provided by you:
	// BR cancoder = 4, FR = 7, FL = 10, BL = 12
	// Drive motor CAN IDs chosen here (adjust to your real motor IDs)
	private static final int FRONT_LEFT_DRIVE_CAN  = 8;  // FL drive NEO
	private static final int FRONT_LEFT_STEER_CAN  = 9;  // FL steer motor (example)
	private static final int FRONT_LEFT_CANCODER   = 10; // FL cancoder

	private static final int FRONT_RIGHT_DRIVE_CAN = 5;  // FR drive NEO
	private static final int FRONT_RIGHT_STEER_CAN = 6;  // FR steer motor (example)
	private static final int FRONT_RIGHT_CANCODER  = 7;  // FR cancoder

	private static final int BACK_LEFT_DRIVE_CAN   = 11;  // BL drive NEO
	private static final int BACK_LEFT_STEER_CAN   = 15;  // BL steer motor (example)
	private static final int BACK_LEFT_CANCODER    = 12; // BL cancoder

	private static final int BACK_RIGHT_DRIVE_CAN  = 2;  // BR drive NEO
	private static final int BACK_RIGHT_STEER_CAN  = 3;  // BR steer motor (example)
	private static final int BACK_RIGHT_CANCODER   = 4;  // BR cancoder (same ID used for cancoder above)

	// steer offsets (degrees) - set per module after measuring
	private static final double FL_STEER_OFFSET_DEG = 0.0;
	private static final double FR_STEER_OFFSET_DEG = 0.0;
	private static final double BL_STEER_OFFSET_DEG = 0.0;
	private static final double BR_STEER_OFFSET_DEG = 0.0;

	private final SwerveModule frontLeft  = new SwerveModule(FRONT_LEFT_DRIVE_CAN,  FRONT_LEFT_STEER_CAN,  FRONT_LEFT_CANCODER,  FL_STEER_OFFSET_DEG);
	private final SwerveModule frontRight = new SwerveModule(FRONT_RIGHT_DRIVE_CAN, FRONT_RIGHT_STEER_CAN, FRONT_RIGHT_CANCODER, FR_STEER_OFFSET_DEG);
	private final SwerveModule backLeft   = new SwerveModule(BACK_LEFT_DRIVE_CAN,   BACK_LEFT_STEER_CAN,   BACK_LEFT_CANCODER,   BL_STEER_OFFSET_DEG);
	private final SwerveModule backRight  = new SwerveModule(BACK_RIGHT_DRIVE_CAN,  BACK_RIGHT_STEER_CAN,  BACK_RIGHT_CANCODER,  BR_STEER_OFFSET_DEG);

	// Kinematics: provide module locations relative to robot center (meters) - adjust to match your robot
	private static final Translation2d FRONT_LEFT_LOCATION = new Translation2d(0.3, 0.25);
	private static final Translation2d FRONT_RIGHT_LOCATION = new Translation2d(0.3, -0.25);
	private static final Translation2d BACK_LEFT_LOCATION = new Translation2d(-0.3, 0.25);
	private static final Translation2d BACK_RIGHT_LOCATION = new Translation2d(-0.3, -0.25);

	private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
			FRONT_LEFT_LOCATION,
			FRONT_RIGHT_LOCATION,
			BACK_LEFT_LOCATION,
			BACK_RIGHT_LOCATION);

	// NavX gyro (MXP SPI)
	private final AHRS navx;

	// Odometry (uses gyro rotation)
	private final SwerveDriveOdometry odometry;

	public SwerveDriveSubsystem()
	{
		AHRS a = null;
		try {
			a = new AHRS(SPI.Port.kMXP);
		} catch (RuntimeException ex) {
			a = null;
		}
		navx = a;

		odometry = new SwerveDriveOdometry(kinematics, getGyroRotation());
	}

	@Override
	public void periodic()
	{
		// Update odometry with current module states and real gyro rotation.
		SwerveModuleState[] states = getModuleStates();
		Rotation2d gyroRotation = getGyroRotation();
		odometry.update(gyroRotation, states[0], states[1], states[2], states[3]);

		// diagnostics
		Pose2d pose = odometry.getPoseMeters();
		SmartDashboard.putNumber("SwervePoseX", pose.getX());
		SmartDashboard.putNumber("SwervePoseY", pose.getY());
		SmartDashboard.putNumber("SwervePoseRot", pose.getRotation().getDegrees());
		if (navx != null) SmartDashboard.putNumber("NavxYaw", navx.getYaw());
	}

	/** Drive helper: xSpeed, ySpeed in m/s, rot in rad/s. If fieldRelative true, speeds are field-relative. */
	public void drive(double xSpeedMetersPerSec, double ySpeedMetersPerSec, double rotRadPerSec, boolean fieldRelative)
	{
		ChassisSpeeds chassisSpeeds = fieldRelative
				? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedMetersPerSec, ySpeedMetersPerSec, rotRadPerSec, getGyroRotation())
				: new ChassisSpeeds(xSpeedMetersPerSec, ySpeedMetersPerSec, rotRadPerSec);

		SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds);
		// limit wheel speeds to Constants.MAX_SPEED
		SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.MAX_SPEED);
		setModuleStates(states);
	}

	/** Helper: returns current gyro rotation as Rotation2d; zero if gyro missing. */
	private Rotation2d getGyroRotation()
	{
		if (navx == null) return new Rotation2d();
		return Rotation2d.fromDegrees(-navx.getYaw());
	}

	/** Set desired module states (in meters/sec and Rotation2d angle). */
	public void setModuleStates(SwerveModuleState[] states)
	{
		if (states == null || states.length != 4) return;

		frontLeft.setDesiredState(states[0]);
		frontRight.setDesiredState(states[1]);
		backLeft.setDesiredState(states[2]);
		backRight.setDesiredState(states[3]);
	}

	/** Returns module states in order FL, FR, BL, BR */
	public SwerveModuleState[] getModuleStates()
	{
		return new SwerveModuleState[] {
				frontLeft.getState(),
				frontRight.getState(),
				backLeft.getState(),
				backRight.getState()
		};
	}

	public Pose2d getPose()
	{
		return odometry.getPoseMeters();
	}

	public void resetOdometry(Pose2d pose)
	{
		odometry.resetPosition(pose, getGyroRotation());
	}

	/** Zero heading on the NavX (if present). */
	public void zeroHeading()
	{
		if (navx != null) navx.zeroYaw();
	}

	/** Returns robot heading in degrees (0..360) from gyro. */
	public double getHeadingDegrees()
	{
		if (navx == null) return 0.0;
		return navx.getYaw();
	}
}