package frc.robot.subsystems.turret;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class rotationSubsystem extends SubsystemBase {
    private final String LIMELIGHT_NAME = "limelight-turret"; 

    private SparkFlex turret_motor1 = new SparkFlex(24, MotorType.kBrushless); 
    
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
    private int Tid; 
    private int targetTagId = 9; // Default to Red Alliance

    public rotationSubsystem() {
        turnController.setTolerance(1.5); // degrees
    }

    @Override
    public void periodic() {
        Tx = LimelightHelpers.getTX(LIMELIGHT_NAME);
        Ty = LimelightHelpers.getTY(LIMELIGHT_NAME);
        Tv = LimelightHelpers.getTV(LIMELIGHT_NAME);
        Tid = (int) LimelightHelpers.getFiducialID(LIMELIGHT_NAME); // Cast to int
      
        // Check alliance color and set the target ID and Limelight Pipeline
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            if (alliance.get() == DriverStation.Alliance.Red) {
                targetTagId = 9;
                LimelightHelpers.setPipelineIndex(LIMELIGHT_NAME, 0); // Assuming Pipeline 0 is Red
            } else {
                targetTagId = 25;
                LimelightHelpers.setPipelineIndex(LIMELIGHT_NAME, 1); // Assuming Pipeline 1 is Blue
            }
        }

        SmartDashboard.putBoolean("Tag Visible", hasTarget());
        SmartDashboard.putNumber("Distance", getDistanceToTarget());
        SmartDashboard.putNumber("Turret Tx", Tx);
        SmartDashboard.putNumber("Target Tag ID", targetTagId);

        runTurretPID();
    }

    public boolean hasTarget() {
        // We only have a valid target if the Limelight sees a tag AND it's the correct ID
        return Tv && (Tid == targetTagId); 
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

    public void stopRotation() {
        turret_motor1.set(0);
    }

    // This replaces your rotateTurret() Command so it can be called sequentially 
    public void runTurretPID() {
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
    }

    public void rotateTurretRight() {
        if (!hasTarget() || !isWithinSoftLimits()) {
            turret_motor1.set(0);
            return;
        }
        turret_motor1.set(0.05);
    }

    public void rotateTurretLeft() {
        if (!hasTarget() || !isWithinSoftLimits()) {
            turret_motor1.set(0);
            return;
        }
        turret_motor1.set(-0.05);
    }

    public void manualRotateRight(){
        turret_motor1.set(.05);
    }
    
    public void manualRotateLeft(){
        turret_motor1.set(-.05);
    }
}