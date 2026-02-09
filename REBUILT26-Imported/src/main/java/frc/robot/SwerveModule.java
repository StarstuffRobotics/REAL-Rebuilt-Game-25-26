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
    @SuppressWarnings("unused")
    private final RelativeEncoder driveEncoder;
    private final SparkClosedLoopController anglePID;
    private final CANcoder absoluteEncoder;

    public SwerveModule(int driveID, int angleID, int canCoderID, double angleOffset) {
        driveMotor = new SparkFlex(driveID, MotorType.kBrushless);
        angleMotor = new SparkFlex(angleID, MotorType.kBrushless);
        
        absoluteEncoder = new CANcoder(canCoderID);
        configureCANcoder(angleOffset);

        angleEncoder = angleMotor.getEncoder();
        driveEncoder = driveMotor.getEncoder();
        anglePID = angleMotor.getClosedLoopController();

        // --- Configure Angle Motor ---
        SparkFlexConfig angleConfig = new SparkFlexConfig();
        angleConfig.closedLoop.pid(Constants.Swerve.angleP, Constants.Swerve.angleI, Constants.Swerve.angleD);
        angleConfig.closedLoop.positionWrappingEnabled(true);
        angleConfig.closedLoop.positionWrappingInputRange(0, 2 * Math.PI);
        
        angleConfig.encoder.positionConversionFactor(Constants.Swerve.STEER_ROTATIONS_TO_RADIANS);
        angleConfig.inverted(true); // MAXSwerve steering is typically inverted

        // --- Configure Drive Motor ---
        SparkFlexConfig driveConfig = new SparkFlexConfig();
        driveConfig.encoder.positionConversionFactor(Constants.Swerve.DRIVE_ROTATIONS_TO_METERS);
        driveConfig.encoder.velocityConversionFactor(Constants.Swerve.DRIVE_ROTATIONS_TO_METERS / 60.0);
        driveConfig.inverted(false); // Change to true if robot drives backward

        // --- Apply Configs ---
        angleMotor.configure(angleConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        resetToAbsolute();
    }

    private void configureCANcoder(double offset) {
        var config = new CANcoderConfiguration();
        config.MagnetSensor.MagnetOffset = offset;
        absoluteEncoder.getConfigurator().apply(config);
    }

    public void resetToAbsolute() {
        // CANcoder returns 0 to 1 rotations. Convert to 0 to 2PI radians.
        double absolutePosition = absoluteEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI;
        angleEncoder.setPosition(absolutePosition);
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromRadians(angleEncoder.getPosition());
    }

    public void setState(SwerveModuleState state) {
        SwerveModuleState optimized = SwerveModuleState.optimize(state, getAngle());
        
        // Use setReference for the drive motor to use the internal PID or just set percentage
        driveMotor.set(optimized.speedMetersPerSecond / Constants.Swerve.MAX_SPEED);
        anglePID.setReference(optimized.angle.getRadians(), ControlType.kPosition);
    }
}