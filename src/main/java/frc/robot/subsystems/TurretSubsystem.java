package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkFlex;

import frc.robot.LimelightHelpers;
import frc.robot.constants; 

public class TurretSubsystem extends SubsystemBase {
    
        //electronics
private RevNeo turret_motor;
private CanCoder turret_encoder;




public enum TurretState {
    IDLE, SPIN, FIND_TARGET, TRACK_TARGET
}

    public TurretSubsystem(){

        turret_motor = new RevNeo(21);
        turret_encoder = new CanCoder(31);
}

    public void periodic(){
        Tx = LimelightHelpers.getTX("limelight-vision");
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

        return(36 - limelightTurretHeight) / Math.tan(angleToGoalRadians);

    


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