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
import edu.wpi.first.math.kinematics.SwerveModulePosition;
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

        SparkFlexConfig angleConfig = new SparkFlexConfig();
        angleConfig.closedLoop
            .pid(Constants.Swerve.angleP, Constants.Swerve.angleI, Constants.Swerve.angleD)
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(0, 2 * Math.PI);
        
        angleConfig.encoder.positionConversionFactor(Constants.Swerve.STEER_ROTATIONS_TO_RADIANS);
        angleConfig.inverted(steeringInverted); 

        SparkFlexConfig driveConfig = new SparkFlexConfig();
        driveConfig.encoder.positionConversionFactor(Constants.Swerve.DRIVE_ROTATIONS_TO_METERS);
        driveConfig.encoder.velocityConversionFactor(Constants.Swerve.DRIVE_ROTATIONS_TO_METERS / 60.0);

        angleMotor.configure(angleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        resetToAbsolute();
    }
//
    public void resetToAbsolute() {
        double absolutePosition = absoluteEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI;
        angleEncoder.setPosition(absolutePosition);
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromRadians(angleEncoder.getPosition());
    }

    public void setState(SwerveModuleState state) {
        // Skip optimize when speed is 0 — optimize can flip the angle 180°
        // for free (since -0 == 0), causing the PID target to flip-flop each cycle
        SwerveModuleState optimized = (state.speedMetersPerSecond == 0)
            ? state
            : SwerveModuleState.optimize(state, getAngle());
        driveMotor.set(optimized.speedMetersPerSecond / Constants.Swerve.MAX_SPEED);

        // SparkMax wrapping is 0 to 2PI
        double targetAngle = optimized.angle.getRadians();
        while (targetAngle < 0) targetAngle += 2 * Math.PI;
        while (targetAngle >= 2 * Math.PI) targetAngle -= 2 * Math.PI;

        anglePID.setReference(targetAngle, ControlType.kPosition);
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveMotor.getEncoder().getPosition(), getAngle());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(driveMotor.getEncoder().getVelocity(), getAngle());
    }
}