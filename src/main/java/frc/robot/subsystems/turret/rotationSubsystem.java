package frc.robot.subsystems.turret;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class rotationSubsystem extends SubsystemBase {
    private final String LIMELIGHT_NAME = "limelight"; 

    private SparkFlex turret_motor1 = new SparkFlex(26, MotorType.kBrushless); 
    
    // Soft limit constants (tune to your robot's physical range)
    private static final double TURRET_MAX_POSITION = 60.0;  // degrees
    private static final double TURRET_MIN_POSITION = -60.0; // degrees

    private final PIDController turnController = new PIDController(
        Constants.LimelightConstants.kTurnP, 
        Constants.LimelightConstants.kTurnI, 
        Constants.LimelightConstants.kTurnD
    );
    
    private double Tx;
    private double Ty;
    private boolean Tv;

    public rotationSubsystem() {
        turnController.setTolerance(1.5); // degrees
    }

    @Override
    public void periodic() {
        Tx = LimelightHelpers.getTX(LIMELIGHT_NAME);
        Ty = LimelightHelpers.getTY(LIMELIGHT_NAME);
        Tv = LimelightHelpers.getTV(LIMELIGHT_NAME);
      
        SmartDashboard.putBoolean("Tag Visible", hasTarget());
        SmartDashboard.putNumber("Distance", getDistanceToTarget());
        SmartDashboard.putNumber("Turret Tx", Tx);
    }
    public boolean hasTarget() {
        return Tv; 
    }
    public double getDistanceToTarget() {
        if (!hasTarget()) return 0.0;
        return calculateDistance(Ty);
    }
    public double calculateDistance(double ty) {
        double totalAngleDegrees = Constants.LimelightConstants.limelightMountAngleDegrees + ty;
        double angleToGoalRadians = Units.degreesToRadians(totalAngleDegrees);
        if (Math.abs(Math.tan(angleToGoalRadians)) < 1e-5) return 0.0;
        return (Units.feetToMeters(Constants.FieldConstants.TARGET_HEIGHT_FEET) - Constants.LimelightConstants.limelightTurretHeight) 
               / Math.tan(angleToGoalRadians);
    }
    
    private boolean isWithinSoftLimits() {
        // Convert encoder position to degrees — adjust the scale factor to match your gearing
        double turretDegrees = turret_motor1.getEncoder().getPosition() * Constants.rotationConstants.ENCODER_DEGREES_PER_ROTATION;
        return turretDegrees > TURRET_MIN_POSITION && turretDegrees < TURRET_MAX_POSITION;
    }

    public Command rotateTurret() {
        return new RunCommand(() -> {
            if (!hasTarget()) {
                turret_motor1.set(0);
                return;
            }

            // Stop if soft limits are exceeded
            if (!isWithinSoftLimits()) {
                turret_motor1.set(0);
                return;
            }

            double rotationSpeed = -turnController.calculate(Tx, 0.0);
            
            // Widened deadband to reduce jitter
            if (Math.abs(Tx) < 2.5) {
                rotationSpeed = 0.0;
            }

            // Clamp output to safe range
            rotationSpeed = Math.max(-0.4, Math.min(0.4, rotationSpeed));

            turret_motor1.set(rotationSpeed);
        }, this);
    }
}