package main.java.frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkFlex;
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

    }
    public void startMotor(double speed){
        turret_motor.set(speed);
    }

    public void stopMotor(){
        turret_motor.stopMotor();
    }

    public double getAngle(){
        turret_encoder.getAbsolutePosition().getValueAsDouble();
    }

    public double getDistance(){

    }
}