package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class hoodSubsystem extends SubsystemBase {
    
    private final Servo m_servoLeft;
    private final Servo m_servoRight;

    public hoodSubsystem() {
        m_servoLeft = new Servo(0); 
        m_servoLeft.setBoundsMicroseconds(2000, 0, 1500, 0, 1000);
        m_servoRight = new Servo(1); 
        m_servoRight.setBoundsMicroseconds(2000, 0, 1500, 0, 1000);
    
    }
  

//lj;kjkl  ;lkjlj lj 

    /** Returns the current angle of the left servo as a proxy for hood angle. */
    public double getHoodAngle() {
        return m_servoLeft.getAngle();
    }

    public void setHoodZero(){
        System.out.println("HOOD min CALLED");
        m_servoLeft.setAngle(0);
        m_servoRight.setAngle(180); // ← inverted
    }
    
    public void setHoodMax(){
        System.out.println("HOOD MAX CALLED");
        m_servoLeft.setAngle(180);
        m_servoRight.setAngle(0); // ← inverted
    }
    
    public void setHoodAngle(double angle) {
        m_servoLeft.setAngle(angle);
        m_servoRight.setAngle(180 - angle); // ← inverted
    }


} 