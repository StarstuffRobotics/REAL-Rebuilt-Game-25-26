package frc.robot;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkFlexConfig;
// Problematic FeedbackSensor import removed
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

    public SwerveModule(int driveID, int angleID, int canCoderID, double angleOffset, boolean steeringInverted) {
        driveMotor = new SparkFlex(driveID, MotorType.kBrushless);
        angleMotor = new SparkFlex(angleID, MotorType.kBrushless);
        absoluteEncoder = new CANcoder(canCoderID);

        CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
        canCoderConfig.MagnetSensor.MagnetOffset = angleOffset;
        absoluteEncoder.getConfigurator().apply(canCoderConfig);

        angleEncoder = angleMotor.getEncoder();
        anglePID = angleMotor.getClosedLoopController();

        // Steering Config
        SparkFlexConfig angleConfig = new SparkFlexConfig();
        
        /* * FIX: We've removed the enum call that was causing the red lines. 
         * By not calling .feedbackSensor(), the Spark Flex defaults to the 
         * Primary Encoder (which is exactly what we want).
         */
        angleConfig.closedLoop
            .pid(Constants.Swerve.angleP, Constants.Swerve.angleI, Constants.Swerve.angleD)
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(0, 2 * Math.PI);
        
        angleConfig.encoder.positionConversionFactor(Constants.Swerve.STEER_ROTATIONS_TO_RADIANS);
        angleConfig.inverted(steeringInverted); 
        angleConfig.smartCurrentLimit(40);

        // Drive Config
        SparkFlexConfig driveConfig = new SparkFlexConfig();
        driveConfig.encoder
            .positionConversionFactor(Constants.Swerve.DRIVE_ROTATIONS_TO_METERS)
            .velocityConversionFactor(Constants.Swerve.DRIVE_ROTATIONS_TO_METERS / 60.0);
        driveConfig.inverted(false);
        driveConfig.smartCurrentLimit(60);

        // Apply and Persist
        angleMotor.configure(angleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        resetToAbsolute();
    }

    public void resetToAbsolute() {
        double absolutePosition = absoluteEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI;
        angleEncoder.setPosition(absolutePosition);
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromRadians(angleEncoder.getPosition());
    }

    public double getAbsolutePosition() {
        return absoluteEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI;
    }

    public double getRelativePosition() {
        return angleEncoder.getPosition();
    }

    public void setState(SwerveModuleState state) {
        SwerveModuleState optimized = SwerveModuleState.optimize(state, getAngle());
        driveMotor.set(optimized.speedMetersPerSecond / Constants.Swerve.MAX_SPEED);
        anglePID.setReference(optimized.angle.getRadians(), ControlType.kPosition);
    }
}