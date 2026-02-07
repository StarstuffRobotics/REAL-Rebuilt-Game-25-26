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

public class SwerveModule {
    private final SparkFlex driveMotor;
    private final SparkFlex angleMotor;
    private final RelativeEncoder angleEncoder;
    private final SparkClosedLoopController anglePID;

    public SwerveModule(int driveID, int angleID) {
        driveMotor = new SparkFlex(driveID, MotorType.kBrushless);
        angleMotor = new SparkFlex(angleID, MotorType.kBrushless);

        angleEncoder = angleMotor.getEncoder();
        anglePID = angleMotor.getClosedLoopController();

        // --- Configure Angle Motor ---
        SparkFlexConfig angleConfig = new SparkFlexConfig();
        
        // Set PID values from your Constants file
        angleConfig.closedLoop.pid(Constants.Swerve.angleP, Constants.Swerve.angleI, Constants.Swerve.angleD);
        
        // Conversion factor: Set this so 1 rotation = 2 * PI radians
        // This accounts for the steering gear ratio (12.8:1 in this example)
        angleConfig.encoder.positionConversionFactor(2 * Math.PI / 12.8); 

        // --- Configure Drive Motor ---
        SparkFlexConfig driveConfig = new SparkFlexConfig();
        // You can add current limits here if needed: driveConfig.smartCurrentLimit(40);

        // --- Apply Configs (Updated for 2026) ---
        // We use kNoPersistParameters to avoid the deprecation warning
        angleMotor.configure(angleConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    /** Returns the current heading of the module */
    public Rotation2d getAngle() {
        return Rotation2d.fromRadians(angleEncoder.getPosition());
    }

    /** Sets the desired state (speed and angle) for the module */
    public void setState(SwerveModuleState state) {
        // Prevent wheel "jitter" when the robot is nearly still
        if (Math.abs(state.speedMetersPerSecond) < 0.01) {
            driveMotor.set(0);
            return;
        }

        // Optimize the state to prevent the wheel from spinning more than 90 degrees
        SwerveModuleState optimized = SwerveModuleState.optimize(state, getAngle());
        
        // Drive speed is normalized (value between -1.0 and 1.0)
        driveMotor.set(optimized.speedMetersPerSecond / Constants.Swerve.MAX_SPEED);
        
        // Set the steering position using the Spark closed-loop controller
        anglePID.setReference(optimized.angle.getRadians(), ControlType.kPosition);
    }
}