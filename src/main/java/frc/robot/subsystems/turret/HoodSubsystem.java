package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HoodSubsystem extends SubsystemBase {
    
    private final Servo m_servo = new Servo(0);

    public void setSpeed() {
        m_servo.set(0.5);
    }

    public void setReversedSpeed(){
        m_servo.set(0.5);
    }

    public void setSpeed(double speed) {
        m_servo.set(speed);
    }


    public void setReversedSpeed(double speed) {
        m_servo.set(-speed);
    }

    public void getHoodAngle() {
        // Implement logic to get the current angle of the hood
    }
}