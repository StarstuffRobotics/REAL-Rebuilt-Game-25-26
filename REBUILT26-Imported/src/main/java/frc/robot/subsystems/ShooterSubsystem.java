package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterSubsystem extends SubsystemBase {
    // Hardware IDs
    private final SparkFlex pivotMotor = new SparkFlex(22, MotorType.kBrushless);//IDK if these can IDs are correct
    private final SparkFlex flywheelMotor = new SparkFlex(24, MotorType.kBrushless);
    
    private final SparkClosedLoopController pivotPID = pivotMotor.getClosedLoopController();
    private final SparkClosedLoopController flywheelPID = flywheelMotor.getClosedLoopController();
    
    private final RelativeEncoder pivotEncoder = pivotMotor.getEncoder();
    private final RelativeEncoder flywheelEncoder = flywheelMotor.getEncoder();

    // Constants
    private final double MIN_ANGLE = 50.0;
    private final double MAX_ANGLE = 75.0;
    private double targetRPM = 0;
    private double targetAngle = 75.0;

    public ShooterSubsystem() {
        SparkFlexConfig config = new SparkFlexConfig();
        
        // Setup Flywheel
        config.closedLoop.p(0.0001).velocityFF(0.00018);
        config.idleMode(SparkFlexConfig.IdleMode.kCoast);
        flywheelMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Setup Pivot
        SparkFlexConfig pivotConfig = new SparkFlexConfig();
        pivotConfig.closedLoop.p(0.1); 
        pivotConfig.idleMode(SparkFlexConfig.IdleMode.kBrake);
        pivotConfig.encoder.positionConversionFactor(1.0); // 1.0 assumes degrees per rotation
        pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void updateShooterTargets(double distance) {
        // Simple placeholder math: as distance increases, angle increases
        // (This replaces the complex math for a moment to ensure code runs)
        this.targetAngle = Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, 50 + (distance * 2)));
        this.targetRPM = 2500 + (distance * 100);

        pivotPID.setReference(targetAngle, ControlType.kPosition);
        flywheelPID.setReference(targetRPM, ControlType.kVelocity);
    }

    public boolean isReady() {
        boolean speedReady = Math.abs(flywheelEncoder.getVelocity() - targetRPM) < 150;
        boolean angleReady = Math.abs(pivotEncoder.getPosition() - targetAngle) < 2.0;
        return speedReady && angleReady;
    }

    public void stop() {
        flywheelMotor.set(0);
        targetRPM = 0;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Actual RPM", flywheelEncoder.getVelocity());
        SmartDashboard.putNumber("Actual Angle", pivotEncoder.getPosition());
    }
}