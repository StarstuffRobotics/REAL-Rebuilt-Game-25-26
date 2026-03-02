package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.LimelightHelpers;
import frc.robot.Constants;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.CANcoder;

public class TurretSubsystem extends SubsystemBase {
    
        //electronics
    private SparkFlex turret_motor1 = new SparkFlex(25, MotorType.kBrushless);
    private SparkFlex turret_motor2 = new SparkFlex(25, MotorType.kBrushless);

    public enum TurretState {
        IDLE, SPIN, FIND_TARGET, TRACK_TARGET
    }

    public TurretSubsystem(){
        double Tx;
        double Ty;
        boolean Tv;
    }

    public void periodic(){
        double Tx = LimelightHelpers.getTX("limelight-vision");
         Ty = LimelightHelpers.getTY("limelight-vision");
         Tv = LimelightHelpers.getTV("limelight-vision");
    }
    public void startMotor(double speed){
        turret_motor.set(speed);
    }

    public void stopMotor(){
        turret_motor.stopMotor();
    }

    public double getAngle(){
        return turret_encoder.getAbsolutePosition().getValueAsDouble();
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