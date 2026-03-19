package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class hoodSubsystem extends SubsystemBase {
    
    private final Servo m_servoLeft;
    private final Servo m_servoRight;

    public hoodSubsystem() {
        m_servoLeft = new Servo(0); 
        //m_servoLeft.setBoundsMicroseconds(2000, 0, 0, 0, 1000);
        m_servoRight = new Servo(1); 
        //m_servoRight.setBoundsMicroseconds(2000, 0, 0, 0, 1000);
    
    }
    
    public void stopHood() {
        m_servoLeft.setSpeed(0);
        m_servoRight.setSpeed(0);
    }

//lj;kjkl  ;lkjlj lj 

    /** Returns the current angle of the left servo as a proxy for hood angle. */
    public double getHoodAngle() {
        return m_servoLeft.getAngle();
    }

    public void setHoodZero(){
        m_servoLeft.set(0);
        m_servoRight.set(0);
    }

    public void setHoodMax(){
        m_servoLeft.set(1);
        m_servoRight.set(1);
    }

    public void setHoodAngle(double angle) {
        m_servoLeft.set(angle);
        m_servoRight.set(angle);
    }


}