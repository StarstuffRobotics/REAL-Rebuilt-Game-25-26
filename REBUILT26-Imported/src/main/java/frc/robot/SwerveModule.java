package frc.robot;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.CANcoderConfiguration;

public class SwerveModule {
    private final SparkFlex driveMotor;
    private final SparkFlex angleMotor;
    private final RelativeEncoder angleEncoder;
    private final SparkClosedLoopController anglePID;
    private final CANcoder absoluteEncoder;

    public SwerveModule(int driveID, int angleID, int canCoderID, double angleOffset) {
        driveMotor = new SparkFlex(driveID, MotorType.kBrushless);
        angleMotor = new SparkFlex(angleID, MotorType.kBrushless);
        absoluteEncoder = new CANcoder(canCoderID);

        // 1. Configure the CANcoder (Offset is in rotations, 0 to 1)
        CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
        canCoderConfig.MagnetSensor.MagnetOffset = angleOffset;
        absoluteEncoder.getConfigurator().apply(canCoderConfig);

        angleEncoder = angleMotor.getEncoder();
        anglePID = angleMotor.getClosedLoopController();

        // 2. Configure Angle Motor (Spark Flex)
        SparkFlexConfig angleConfig = new SparkFlexConfig();
        angleConfig.closedLoop
            .feedbackSensor(com.revrobotics.spark.FeedbackSensor.kPrimaryEncoder)
            .pid(Constants.Swerve.angleP, Constants.Swerve.angleI, Constants.Swerve.angleD)
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(0, 2 * Math.PI);
        
        angleConfig.encoder.positionConversionFactor(Constants.Swerve.STEER_ROTATIONS_TO_RADIANS);
        angleConfig.inverted(true); // Standard for MAXSwerve
        angleConfig.smartCurrentLimit(40); // Protect the motor from stalls

        // 3. Configure Drive Motor (Spark Flex)
        SparkFlexConfig driveConfig = new SparkFlexConfig();
        driveConfig.encoder
            .positionConversionFactor(Constants.Swerve.DRIVE_ROTATIONS_TO_METERS)
            .velocityConversionFactor(Constants.Swerve.DRIVE_ROTATIONS_TO_METERS / 60.0);
        
        driveConfig.inverted(false);
        driveConfig.smartCurrentLimit(60); // Flex/Vortex can handle high current, but start safe
        driveConfig.openLoopRampRate(0.25); // Smooths out rapid acceleration

        // 4. Apply Configs to Hardware
        // ResetMode.kResetSafeParameters ensures a clean state before applying
        // PersistMode.kPersistParameters saves settings so they survive a brownout
        angleMotor.configure(angleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // 5. Sync the internal encoder with the absolute CANcoder
        resetToAbsolute();
    }

    public void resetToAbsolute() {
        // Convert CANcoder rotations (0-1) to Radians (0-2PI)
        double absolutePosition = absoluteEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI;
        angleEncoder.setPosition(absolutePosition);
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromRadians(angleEncoder.getPosition());
    }

    public void setState(SwerveModuleState state) {
        // Optimize avoids spinning the wheel more than 90 degrees
        SwerveModuleState optimized = SwerveModuleState.optimize(state, getAngle());
        
        // Scale velocity to percentage (-1.0 to 1.0) for the drive motor
        driveMotor.set(optimized.speedMetersPerSecond / Constants.Swerve.MAX_SPEED);
        
        // Use the on-board PID controller for the steering angle
        anglePID.setReference(optimized.angle.getRadians(), ControlType.kPosition);
    }
    // Add these to SwerveModule.java
public double getAbsolutePosition() {
    return absoluteEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI;
}

public double getRelativePosition() {
    return angleEncoder.getPosition();
}
}