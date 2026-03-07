package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj.Servo;

public class hoodSubsystem {
    private final Servo linearServo1 = new Servo(0);
    private final Servo linearServo2 = new Servo(1);
    

    private final double[] estimatedposes = {0.0,.28,.4,.64,.68,.8,.84,.88,1.0}; //these are the estimated servo positions for each hood angle from 15 to 40 degrees in 2.5 degree increments


    private int i = 0;

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

    public void cycleHoodAngle(int mod){
    
        i+=mod;

        if (i<0||i>=estimatedposes.length){
            i = 0; 
        }else{

            double pos = estimatedposes[i];

            linearServo1.setPosition(pos);
            linearServo2.setPosition(pos);

        }
    }

    public void setHoodAngleCustom(double hoodAngle){
        if(hoodAngle < 15){
            hoodAngle = 15;
        }
        else if(hoodAngle > 40){
            hoodAngle = 40;
        }

        double servoPosition = (hoodAngle - 15) / 25.0;


        linearServo1.setPosition(getHoodAngle()+servoPosition); //might need to change for the right percentage
        linearServo2.setPosition(getHoodAngle()+servoPosition); //might need to change for the right percentage
    }

    public void stopHood(){
        linearServo1.setPosition(0);
        linearServo2.setPosition(0);
    }

    public void setHoodAngle(double hoodAngle){
        linearServo1.setSpeed(hoodAngle/100); //might need to change for the right percentage
        linearServo2.setSpeed(hoodAngle/100); //might need to change for the right percentage
    }
    


}
