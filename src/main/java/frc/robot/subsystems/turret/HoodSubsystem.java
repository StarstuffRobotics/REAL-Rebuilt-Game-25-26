package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HoodSubsystem extends SubsystemBase {
    
    private final Servo m_servo;

    public HoodSubsystem() {
        m_servo = new Servo(0); 
        m_servo.setBoundsMicroseconds(2100, 1508, 1500, 1492, 900);
    }
    
    public void stopHood() {
        m_servo.setSpeed(0);
    }

    // public void setSpeed() {
    //     m_servo.setSpeed(0.3);
        
    // }

    // public void setReversedSpeed(){
    //     m_servo.setSpeed(-0.3);
    // }

    // public void setSpeed(double speed) {
    //     m_servo.setSpeed(speed);
    // }


    // public void setReversedSpeed(double speed) {
    //     m_servo.setSpeed(-speed);
    // }

    public void getHoodAngle() {
        // Implement logic to get the current angle of the hood
    }

    public void setHoodZero(){
        m_servo.setAngle(0);
    }

    public void setHoodMax(){
        m_servo.setAngle(60);
    }

    public void setHoodAngle(double angle) {
        m_servo.setAngle(angle);
    }


}