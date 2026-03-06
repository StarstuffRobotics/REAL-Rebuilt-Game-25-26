package frc.robot.subsystems.turret;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class shooterSubsystem extends SubsystemBase {
    
        //electronics
    private SparkFlex turret_motor1 = new SparkFlex(25, MotorType.kBrushless);
    private SparkFlex turret_motor2 = new SparkFlex(25, MotorType.kBrushless);
   
    private static double limelightMountAngleDegrees;
    private static double angleToGoalDegrees;
    private static double angleToGoalRadians;
    private double Tx;
    private double Ty;
    private boolean Tv;
    private static final double GRAVITY = 32.17;

    private static final double K_CALIBRATION = 0.85;

    private static final double WHEEL_RADIUS_FT = 0.166;

    private static final double MIN_ANGLE_DEG = 15.0;
    private static final double MAX_ANGLE_DEG = 40.0;

    private static final double LAUNCH_HEIGHT_FT = 1.775;
    private static final double TARGET_HEIGHT_FT = 6.0;
    private static final double DELTA_H = TARGET_HEIGHT_FT - LAUNCH_HEIGHT_FT;

    
    private static final double MAX_EXIT_VELOCITY_FT_PER_SEC = 31.54;



    public enum TurretState {
        IDLE, SPIN, FIND_TARGET, TRACK_TARGET
    }

    public shooterSubsystem(){
     
    }

    public void periodic(){
        Tx = LimelightHelpers.getTX("limelight-vision");
        Ty = LimelightHelpers.getTY("limelight-vision");
        Tv = LimelightHelpers.getTV("limelight-vision");
    }

    public void startMotor(){
        double speed = calculateTargetRPM(getDistanceToHub(), getHoodAngle())/5700; 
        turret_motor1.set(speed);
        turret_motor2.set(speed);
    }

    public void stopMotor(){
        turret_motor1.stopMotor();
        turret_motor2.stopMotor();
    }

    public double getHoodAngle(){
        //return turret_encoder.getAbsolutePosition().getValueAsDouble();
        return 0 ;//change for the base angle of the hood when it is down 
    }

    public double getDistanceToHub(){
            limelightMountAngleDegrees = Constants.LimelightConstants.limelightMountAngleDegrees;
            angleToGoalDegrees = limelightMountAngleDegrees + Ty;
            angleToGoalRadians = Math.toRadians(angleToGoalDegrees);

            return(36 - Ty) / Math.tan(angleToGoalRadians);//do we need to change this so that it will not find distece with y axis in there?
    }

    public static double calculateTargetRPM(double distanceFeet, double hoodAngleDegrees) {

        if (distanceFeet <= 0) return -1;

        double clampedAngle = Math.max(MIN_ANGLE_DEG,
                            Math.min(MAX_ANGLE_DEG, hoodAngleDegrees));

        double theta = Math.toRadians(clampedAngle);
        
        double numerator = GRAVITY * Math.pow(distanceFeet, 2);

        double denominator =
                2 * Math.pow(Math.cos(theta), 2)
                * (distanceFeet * Math.tan(theta) - DELTA_H);

        
        if (denominator <= 0.01) return -1;
        

        double v0 = Math.sqrt(numerator / denominator);
        
        if (v0 > MAX_EXIT_VELOCITY_FT_PER_SEC) {
            v0 = MAX_EXIT_VELOCITY_FT_PER_SEC;
        }
        
        double rpm =
                ((v0 / K_CALIBRATION) / WHEEL_RADIUS_FT)
                * 60 / (2 * Math.PI);

        return rpm;
    }
}