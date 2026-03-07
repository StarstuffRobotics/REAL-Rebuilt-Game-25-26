package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj.Servo;
import frc.robot.Constants.hoodConstants;

public class hoodSubsystem {
    private final Servo linearServo1 = new Servo(hoodConstants.SERVO_PORT);
    private final Servo linearServo2 = new Servo(hoodConstants.SERVO_PORT);
    

    ///
    /// 
    /// Note for later : hoodAngle ≈ 14 + 1.6 × distanceFeet
    /// that is an aproximation for the deiered hood angle based on distance
    /// here is an aprocimate table:
    ///2–4 ft → 15°–22°
    // 5–8 ft → 25°–31°
    // 9–12 ft → 32°–35°
    // 13–15 ft → 36°–37°
    /// 
    /// 
    

    public void findOptomalHoodAngle(double distanceFeet){//need to add in percent error
        double hoodAngle = 14 + 1.6 * distanceFeet;
        setHoodAngle(hoodAngle);
    }

    public double getHoodAngle(){
        
        double servoPosition = linearServo1.getPosition(); 
        double hoodAngle = (servoPosition * 100)/25; 
        return hoodAngle+15; // Adjusting for the 15° offset
    }

    public void setHoodAngleCustom(double hoodAngle){
        if(hoodAngle < 15){
            hoodAngle = 15;
        }
        else if(hoodAngle > 40){
            hoodAngle = 40;
        }
        linearServo1.setPosition(hoodAngle/100);
        linearServo2.setPosition(hoodAngle/100);
    }

    public void stopHood(){
        linearServo1.setPosition(0);
        linearServo2.setPosition(0);
    }

    public void setHoodAngle(double hoodAngle){
        linearServo1.setPosition(hoodAngle/100); //might need to change for the right percentage
        linearServo2.setPosition(hoodAngle/100); //might need to change for the right percentage
    }
    


}
