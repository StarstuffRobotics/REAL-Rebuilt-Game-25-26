package frc.robot.commands.turret;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.turret.LinearServo;


public class hoodCommands extends SubsystemBase {
    LinearServo hood1;
    LinearServo hood2;

    public double distance = 0 ;
   
    
   
    public hoodCommands(){
        this.hood1 = LinearServo.getHood2(); // Example parameters: channel 1, length 100mm, speed 50mm/s
        this.hood2 = LinearServo.getHood1(); // Example parameters: channel 2, length 100mm, speed 50mm/s
    }

    @Override
    public void periodic() {
        hood1.updateCurPos();  
        hood2.updateCurPos();
    }

    public void setPosition(double distance){
        hood1.setPosition(distance);
        hood2.setPosition(distance);
    }

    public void updateHood(){
        hood1.updateCurPos();
        hood2.updateCurPos();
    }

    public double getHoodAngle(){
        return hood1.getAngle();
    }

    public void hoodUp(){
        hood1.setPosition(getHoodAngle() + 3);
        hood2.setPosition(getHoodAngle() + 3);
    }
    public void hoodDown(){
        hood1.setPosition(getHoodAngle() - 3);
        hood2.setPosition(getHoodAngle() - 3);
    }

    public void hoodZero(){
        hood1.setPosition(0);
        hood2.setPosition(0);
    }

    public void updateCurPos(){
        hood1.updateCurPos();
        hood2.updateCurPos();
    }
    
}
