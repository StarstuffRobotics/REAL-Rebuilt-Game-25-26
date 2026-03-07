package frc.robot.commands.turret;
import frc.robot.subsystems.turret.LinearServo;


public class hoodCommands {
    LinearServo hood;

    public double distance = 0 ;
   
    public hoodCommands(){
        this.hood = new LinearServo(1, 100, 50); // Example parameters: channel 1, length 100mm, speed 50mm/s
        
    }

    public void setPosition(double distance){
        hood.setPosition(distance);
    }

    public void updateHood(){
        hood.updateCurPos();
    }

    public double getHoodAngle(){
        return hood.getAngle();
    }

    public void hoodUp(){
        hood.setPosition(hood.getAngle() + 0.1);
    }
    public void hoodDown(){
        hood.setPosition(hood.getAngle() - 0.1);
    }

    public void hoodZero(){
        hood.setPosition(0);
    }
   
    
}
