package frc.robot.subsystems.turret;

import java.util.Set;

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

    private final SparkFlex turret_motor1 = new SparkFlex(24, MotorType.kBrushless);

    // Soft limit constants
    private static final double TURRET_MAX_POSITION = 60.0;  // degrees
    private static final double TURRET_MIN_POSITION = -60.0; // degrees

   
    // Based on field map: Red hub = 2,3,4,5,8,9,10,11 | Blue hub = 18,19,20,21,24,25,26,27
    private static final Set<Integer> RED_HUB_TAGS  = Set.of(2, 3, 4, 5, 8, 9, 10, 11);
    private static final Set<Integer> BLUE_HUB_TAGS = Set.of(18, 19, 20, 21, 24, 25, 26, 27);

    private final PIDController turnController = new PIDController(
        Constants.LimelightConstants.kTurnP,
        Constants.LimelightConstants.kTurnI,
        Constants.LimelightConstants.kTurnD
    );

    private double Tx;
    private double Ty;
    private boolean Tv;
    private int Tid;

   
    private Set<Integer> validHubTags = RED_HUB_TAGS;

    public rotationSubsystem() {
        turret_motor1.getEncoder().setPosition(0);
        turnController.setTolerance(1.5); // degrees
    }

    @Override
    public void periodic() {
        Tx  = LimelightHelpers.getTX(LIMELIGHT_NAME);
        Ty  = LimelightHelpers.getTY(LIMELIGHT_NAME);
        Tv  = LimelightHelpers.getTV(LIMELIGHT_NAME);
        Tid = (int) LimelightHelpers.getFiducialID(LIMELIGHT_NAME);

        
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            if (alliance.get() == DriverStation.Alliance.Red) {
                validHubTags = RED_HUB_TAGS;
                LimelightHelpers.setPipelineIndex(LIMELIGHT_NAME, 0);
            } else {
                validHubTags = BLUE_HUB_TAGS;
                LimelightHelpers.setPipelineIndex(LIMELIGHT_NAME, 1);
            }
        }else{
            //defults to red on not allince detected.
            validHubTags = RED_HUB_TAGS;
            LimelightHelpers.setPipelineIndex(LIMELIGHT_NAME, 0);
        }

        SmartDashboard.putBoolean("Hub Tag Visible", hasTarget());
        SmartDashboard.putNumber("Distance",         getDistanceToTarget());
        SmartDashboard.putNumber("Turret Tx",        Tx);
        SmartDashboard.putNumber("Seen Tag ID",      Tid);

        
    }

    
    public boolean hasTarget() {
        return Tv && validHubTags.contains(Tid);
    }

    public double getDistanceToTarget() {
        if (!hasTarget()) return 0.0;
        return calculateDistance(Ty);
    }

    public double calculateDistance(double ty) {
        double totalAngleDegrees  = Constants.LimelightConstants.limelightMountAngleDegrees + ty;
        double angleToGoalRadians = Units.degreesToRadians(totalAngleDegrees);
        if (Math.abs(Math.tan(angleToGoalRadians)) < 1e-5) return 0.0;
        return (Units.feetToMeters(Constants.FieldConstants.TARGET_HEIGHT_FEET)
                - Constants.LimelightConstants.limelightTurretHeight)
               / Math.tan(angleToGoalRadians);
    }

    private boolean isWithinSoftLimits() {
        double turretDegrees = turret_motor1.getEncoder().getPosition()
                               * Constants.rotationConstants.ENCODER_DEGREES_PER_ROTATION;
        return turretDegrees > TURRET_MIN_POSITION && turretDegrees < TURRET_MAX_POSITION;
    }

    public void stopRotation() {
        turret_motor1.set(0);
    }

    public void runTurretPID() {
        // Do nothing if we don't see a tag from the outher allinece.
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

        
        if (Math.abs(Tx) < 2.5) {
            rotationSpeed = 0.0;
        }

        // Clamp output to safe range
        rotationSpeed = Math.max(-0.4, Math.min(0.4, rotationSpeed));

        turret_motor1.set(rotationSpeed);
    }

    public void rotateTurretRight() {
        
        turret_motor1.set(0.05);
    }

    public void rotateTurretLeft() {
        
        turret_motor1.set(-0.05);
    }

    public void manualRotateRight() {
        turret_motor1.set(0.05);
    }

    public void manualRotateLeft() {
        turret_motor1.set(-0.05);
    }
}