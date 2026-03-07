package frc.robot.subsystems.turret;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.LimelightHelpers;
import frc.robot.commands.turret.hoodCommands;

public class shooterSubsystem extends SubsystemBase {

    // Electronics - FIX: give each motor a unique CAN ID
    private SparkFlex turret_motor1 = new SparkFlex(25, MotorType.kBrushless);
    private SparkFlex turret_motor2 = new SparkFlex(26, MotorType.kBrushless); // was 25, now 26
    private boolean turret_motorsOn = false;

    private SparkClosedLoopController motor1Controller = turret_motor1.getClosedLoopController();
    private SparkClosedLoopController motor2Controller = turret_motor2.getClosedLoopController();

    private hoodCommands hood; 

    private static double limelightMountAngleDegrees;
    private double Tx;
    private double Ty;
    private boolean Tv;

    private static final double GRAVITY = 32.17; // ft/s²

    private static final double K_CALIBRATION = 0.85;
    private static final double WHEEL_RADIUS_FT = 0.166;

    private static final double MIN_ANGLE_DEG = 15.0;
    private static final double MAX_ANGLE_DEG = 40.0;

    // Height of limelight above ground (ft)
    private static final double LIMELIGHT_HEIGHT_FT = 1.775;

    // Height of the hopper opening above ground (ft) — hopper is ABOVE the robot
    private static final double TARGET_HEIGHT_FT = 6.0;

    // Positive because hopper is higher than the limelight
    private static final double DELTA_H = TARGET_HEIGHT_FT - LIMELIGHT_HEIGHT_FT; // = 4.225 ft

    private static final double MAX_EXIT_VELOCITY_FT_PER_SEC = 31.54;

    public enum TurretState {
        IDLE, SPIN, FIND_TARGET, TRACK_TARGET
    }

    public shooterSubsystem() {
        // TODO: Configure PID gains on motor1Controller and motor2Controller here
    }

    @Override
    public void periodic() {
        Tx = LimelightHelpers.getTX("limelight-vision");
        Ty = LimelightHelpers.getTY("limelight-vision");
        Tv = LimelightHelpers.getTV("limelight-vision");
    }

    public void shooterOnOff() {
        if(!turret_motorsOn){
            if (!Tv) return; // No target visible, don't spin

            double distance = getFixedDistanceToHub(); // Use Limelight distance calculation
            double rpm = calculateTargetRPM(distance, getHoodAngle());

            double speed = rpm/6784.0; // Convert RPM to percentage of max speed (assuming 6784 RPM is max)

            if (speed < 0) return; // Invalid shot

            // Use closed-loop RPM control instead of open-loop percentage
            turret_motor1.set(speed);
            turret_motor2.set(-speed);
            turret_motorsOn = true;
        }else{
            turret_motor1.stopMotor();
            turret_motor2.stopMotor();
            turret_motorsOn = false;
        }
    }


    public void shooterOnOff(double speed){
        if(!turret_motorsOn){
            turret_motor1.set(speed);
            turret_motor2.set(-speed);
            turret_motorsOn = true;
        } else {
            turret_motor1.stopMotor();
            turret_motor2.stopMotor();
            turret_motorsOn = false;
        }
    }

    public void shooterStop() {
        turret_motor1.stopMotor();
        turret_motor2.stopMotor();
        turret_motorsOn = false;
    }

    public void shooterReverse( ){
        turret_motor1.set(-ShooterConstants.kShooterSpeedReverse);
        turret_motor2.set(ShooterConstants.kShooterSpeedReverse);
    }

    public void shooterReverse(double speed ){
        turret_motor1.set(ShooterConstants.kShooterSpeedReverse);
        turret_motor2.set(-ShooterConstants.kShooterSpeedReverse);
    }

    
    public double getHoodAngle() {
        return hood.getHoodAngle(); // Replace with actual encoder reading
    }

    /**
     * Uses the standard Limelight distance formula:
     *   distance = (targetHeight - limelightHeight) / tan(mountAngle + ty)
     *
     * This correctly accounts for the hopper being above the robot.
     */
    public double getDistanceToHub() {
        limelightMountAngleDegrees = Constants.LimelightConstants.limelightMountAngleDegrees;

        double angleToGoalDegrees = limelightMountAngleDegrees + Ty;
        double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);

        // FIXED: use height difference in numerator, not raw Ty
        return DELTA_H / Math.tan(angleToGoalRadians);
    }
    public double getFixedDistanceToHub(){
        return 15.0;
    }

    /**
     * Calculates the required wheel RPM to reach a target that is ABOVE the robot.
     * Uses projectile motion: v0 = sqrt( g*d^2 / (2*cos²θ * (d*tanθ - Δh)) )
     * DELTA_H is positive, meaning the target is higher than the launch point.
     */
    public static double calculateTargetRPM(double distanceFeet, double hoodAngleDegrees) {

        if (distanceFeet <= 0) return -1;

        double clampedAngle = Math.max(MIN_ANGLE_DEG, Math.min(MAX_ANGLE_DEG, hoodAngleDegrees));
        double theta = Math.toRadians(clampedAngle);

        double numerator = GRAVITY * Math.pow(distanceFeet, 2);

        // d*tanθ must be > DELTA_H, otherwise the angle is too shallow to reach the target
        double denominator = 2 * Math.pow(Math.cos(theta), 2)
                           * (distanceFeet * Math.tan(theta) - DELTA_H);

        if (denominator <= 0.01) return -1; // Angle too shallow, can't reach hopper

        double v0 = Math.sqrt(numerator / denominator);

        if (v0 > MAX_EXIT_VELOCITY_FT_PER_SEC) {
            v0 = MAX_EXIT_VELOCITY_FT_PER_SEC;
        }

        double rpm = ((v0 / K_CALIBRATION) / WHEEL_RADIUS_FT) * (60.0 / (2 * Math.PI));

        return rpm;
    }
}