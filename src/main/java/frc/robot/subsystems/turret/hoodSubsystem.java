package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj.Servo;
import frc.robot.Constants.hoodConstatnts;

public class hoodSubsystem {
    private final Servo linearServo1 = new Servo(hoodConstatnts.SERVO_PORT);
    private final Servo linearServo2 = new Servo(hoodConstatnts.SERVO_PORT);
    

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

    public void setHoodAngle(double hoodAngle){
        linearServo1.setPosition(hoodAngle/100);
        linearServo2.setPosition(hoodAngle/100);
    }
    
}
