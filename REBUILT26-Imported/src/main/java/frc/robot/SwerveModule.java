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
        configureCANcoder(angleOffset);

        angleEncoder = angleMotor.getEncoder();
        anglePID = angleMotor.getClosedLoopController();

        // --- Configure Angle Motor ---
        SparkFlexConfig angleConfig = new SparkFlexConfig();
        angleConfig.closedLoop.pid(Constants.Swerve.angleP, Constants.Swerve.angleI, Constants.Swerve.angleD);
        
        angleConfig.closedLoop.positionWrappingEnabled(true);
        angleConfig.closedLoop.positionWrappingInputRange(0,2 * Math.PI);

        // Conversion factor: Set this so 1 rotation = 2 * PI radians
        // This accounts for the steering gear ratio (12.8:1 in this example)
        double gearRatio = 12.8;
        angleConfig.encoder.positionConversionFactor(2 * Math.PI / gearRatio); 

        // --- Configure Drive Motor ---
        SparkFlexConfig driveConfig = new SparkFlexConfig();
        // You can add current limits here if needed: driveConfig.smartCurrentLimit(40);

        // --- Apply Configs (Updated for 2026) ---
        // We use kNoPersistParameters to avoid the deprecation warning
        angleMotor.configure(angleConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        resetToAbsolute();
    }

    private void configureCANcoder(double offset){
        var config = new CANcoderConfiguration();
        config.MagnetSensor.MagnetOffset = offset;
        absoluteEncoder.getConfigurator().apply(config);
    }

    private void resetToAbsolute(){
        double absolutePosition = absoluteEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI;
        angleEncoder.setPosition(absolutePosition);
    }

    /** Returns the current heading of the module */
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