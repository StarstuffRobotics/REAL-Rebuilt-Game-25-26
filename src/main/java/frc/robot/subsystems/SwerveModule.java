// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package main.java.frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

/**
 * Minimal Swerve Module:
 * - Drive: CANSparkMax (NEO), uses RelativeEncoder.getVelocity() (RPM) -> m/s conversion.
 * - Steer: CANSparkMax for motor output, CANCoder for absolute angle feedback (degrees).
 * - Steering uses WPILib PIDController in position mode (output applied as motor percent).
 *
 * Notes:
 * - Tune the PID gains, gear ratios, and wheel diameter for your robot.
 * - steerOffsetDegrees is applied to the CANCoder reading (use your mount offsets).
 */
public class SwerveModule
{
	private final CANSparkMax driveMotor;
	private final CANSparkMax steerMotor;
	private final RelativeEncoder driveEncoder;
	private final CANCoder cancoder;

	// Steer PID (tune)
	private final PIDController steerPid = new PIDController(0.1, 0.0, 0.0);

	// Hardware-specific conversions (REPLACE with correct values)
	private static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4.0); // you specified 4"
	private static final double DRIVE_GEAR_RATIO = 6.75; // motor rotations -> wheel rotations (placeholder)
	// if DRIVE_GEAR_RATIO = motorRotations / wheelRotations, use accordingly. Adjust as needed.

	// If your steer motor has an internal encoder, we prefer absolute CANCoder for angle feedback.
	private final double steerOffsetRad;

	public SwerveModule(int driveCanId, int steerMotorCanId, int cancoderCanId, double steerOffsetDegrees)
	{
		this.driveMotor = new CANSparkMax(driveCanId, MotorType.kBrushless);
		this.steerMotor = new CANSparkMax(steerMotorCanId, MotorType.kBrushless);

		this.driveEncoder = driveMotor.getEncoder();
		this.cancoder = new CANCoder(cancoderCanId);

		this.steerOffsetRad = Math.toRadians(steerOffsetDegrees);

		// Basic zeroing / safety
		try {
			driveEncoder.setPosition(0.0);
		} catch (Exception ignored) {}

		// Configure PID continuous input for -pi..pi
		steerPid.enableContinuousInput(-Math.PI, Math.PI);
	}

	/** Returns current absolute steer angle in radians, adjusted by configured offset, normalized to -pi..pi. */
	public double getSteerAngleRad()
	{
		double deg = 0.0;
		try {
			deg = cancoder.getAbsolutePosition(); // degrees 0..360
		} catch (Exception e) {
			// fallback if CANCoder not present
			deg = 0.0;
		}
		double rad = Math.toRadians(deg) - steerOffsetRad;
		return normalizeAngleRadians(rad);
	}

	/** Returns drive wheel linear velocity in meters per second. */
	public double getDriveVelocityMetersPerSecond()
	{
		// Spark MAX RelativeEncoder.getVelocity() -> RPM
		double rpm = 0.0;
		try {
			rpm = driveEncoder.getVelocity();
		} catch (Exception e) {
			rpm = 0.0;
		}
		double motorRps = rpm / 60.0;
		// motorRps -> wheelRps depending on gear ratio: wheelRps = motorRps / DRIVE_GEAR_RATIO
		double wheelRps = motorRps / DRIVE_GEAR_RATIO;
		double wheelCircum = Math.PI * WHEEL_DIAMETER_METERS;
		return wheelRps * wheelCircum;
	}

	/** Apply the desired state (m/s and angle). */
	public void setDesiredState(SwerveModuleState desiredState)
	{
		// Optimize state to minimize steering movement
		SwerveModuleState optimized = SwerveModuleState.optimize(desiredState, new Rotation2d(getSteerAngleRad()));

		// DRIVE: map desired speed to percent [-1..1] relative to Constants.MAX_SPEED
		double speedFraction = 0.0;
		if (Constants.MAX_SPEED > 0.0) {
			speedFraction = optimized.speedMetersPerSecond / Constants.MAX_SPEED;
		}
		driveMotor.set(clamp(speedFraction, -1.0, 1.0));

		// STEER: compute error between current and desired angle and run PID
		double targetRad = optimized.angle.getRadians();
		double currentRad = getSteerAngleRad();
		double output = steerPid.calculate(currentRad, targetRad);
		steerMotor.set(clamp(output, -1.0, 1.0));
	}

	/** Returns current SwerveModuleState (velocity m/s and Rotation2d). */
	public SwerveModuleState getState()
	{
		return new SwerveModuleState(getDriveVelocityMetersPerSecond(), new Rotation2d(getSteerAngleRad()));
	}

	// --- utilities ---
	private static double clamp(double v, double lo, double hi)
	{
		return Math.max(lo, Math.min(hi, v));
	}

	private static double normalizeAngleRadians(double angle)
	{
		double a = angle;
		while (a > Math.PI) a -= 2.0 * Math.PI;
		while (a <= -Math.PI) a += 2.0 * Math.PI;
		return a;
	}
}