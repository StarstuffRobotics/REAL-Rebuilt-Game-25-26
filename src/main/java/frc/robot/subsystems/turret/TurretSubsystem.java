package frc.robot.subsystems.turret;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class TurretSubsystem extends SubsystemBase {
    
        //electronics
    private SparkFlex turret_motor1 = new SparkFlex(25, MotorType.kBrushless);
    private SparkFlex turret_motor2 = new SparkFlex(25, MotorType.kBrushless);

    private double Tx;
    private double Ty;
    private boolean Tv;

    public enum TurretState {
        IDLE, SPIN, FIND_TARGET, TRACK_TARGET
    }

    public TurretSubsystem(){
        
    }

    public void periodic(){
        double Tx = LimelightHelpers.getTX("limelight-vision");
         Ty = LimelightHelpers.getTY("limelight-vision");
         Tv = LimelightHelpers.getTV("limelight-vision");
    }
    public void startMotor(double speed){
        turret_motor1.set(speed);
        turret_motor2.set(speed);
    }

    public void stopMotor(){
        turret_motor1.stopMotor();
        turret_motor1.stopMotor();
    }

    public double getAngle(){
        //return turret_encoder.getAbsolutePosition().getValueAsDouble(); need to figure this out
        return 0;
    }

    public double getDistance(){
        double limelightMountAngleDegrees =0;
        double angleToGoalDegrees = limelightMountAngleDegrees + Ty;
        double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);

        return(36 - Ty) / Math.tan(angleToGoalRadians);

    


    }

    public double getTargetAngle(){
        double distance = getDistance();
        double TxRAD = Math.toRadians(Tx);

        double targetX = Tx + distance ;

        return 0;
    }

    public static double calculateVelocity(double g, double d, double thetaDegrees, double h0, double h) {
    
    // Convert degrees to radians
    double theta = Math.toRadians(thetaDegrees);

    double numerator = g * Math.pow(d, 2);

    double denominator = 2 * Math.pow(Math.cos(theta), 2) 
                         * (d * Math.tan(theta) + h0 - h);

    return Math.sqrt(numerator / denominator);
}
}