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

// Import CTRE Phoenix 6 for CANcoder
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.CANcoderConfiguration;

public class SwerveModule {
    private final SparkFlex driveMotor;
    private final SparkFlex angleMotor;
    private final RelativeEncoder angleEncoder;
    private final SparkClosedLoopController anglePID;
    
    // New: The Absolute Encoder
    private final CANcoder absoluteEncoder;

    public SwerveModule(int driveID, int angleID, int canCoderID, double angleOffset) {
        driveMotor = new SparkFlex(driveID, MotorType.kBrushless);
        angleMotor = new SparkFlex(angleID, MotorType.kBrushless);

        // Initialize CANcoder
        absoluteEncoder = new CANcoder(canCoderID);
        configureCANcoder(angleOffset);

        angleEncoder = angleMotor.getEncoder();
        anglePID = angleMotor.getClosedLoopController();

        // --- Configure Angle Motor ---
        SparkFlexConfig angleConfig = new SparkFlexConfig();
        angleConfig.closedLoop.pid(Constants.Swerve.angleP, Constants.Swerve.angleI, Constants.Swerve.angleD);
        
        // Wrap the PID input so the motor knows 2*PI is the same as 0
        angleConfig.closedLoop.positionWrappingEnabled(true);
        angleConfig.closedLoop.positionWrappingInputRange(0, 2 * Math.PI);

        // Conversion factor: 1 motor rotation = (2 * PI / gear ratio) radians
        // SDS MK4i is usually 150/7:1 or 12.8:1. Double check your specific ratio!
        double gearRatio = 12.8; 
        angleConfig.encoder.positionConversionFactor(2 * Math.PI / gearRatio); 

        // --- Apply Configs ---
        angleMotor.configure(angleConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        driveMotor.configure(new SparkFlexConfig(), ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        // --- SEED THE ENCODER ---
        // This makes the Spark Flex internal encoder match the physical wheel position
        resetToAbsolute();
    }

    private void configureCANcoder(double offset) {
        var config = new CANcoderConfiguration();
        config.MagnetSensor.MagnetOffset = offset;
        absoluteEncoder.getConfigurator().apply(config);
    }

    public void resetToAbsolute() {
        // Get absolute position in rotations and convert to radians
        double absolutePosition = absoluteEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI;
        angleEncoder.setPosition(absolutePosition);
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromRadians(angleEncoder.getPosition());
    }

    public void setState(SwerveModuleState state) {
        // Optimize the state
        SwerveModuleState optimized = SwerveModuleState.optimize(state, getAngle());
        
        driveMotor.set(optimized.speedMetersPerSecond / Constants.Swerve.MAX_SPEED);
        anglePID.setReference(optimized.angle.getRadians(), ControlType.kPosition);
    }
}