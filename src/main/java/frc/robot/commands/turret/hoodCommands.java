package frc.robot.commands.turret;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.turret.LinearServo;


public class hoodCommands extends SubsystemBase {
    // ✅ Create instances here with CORRECT ports (8 and 9)
    private final LinearServo hood1 = new LinearServo(8, 100, 50);
    private final LinearServo hood2 = new LinearServo(9, 100, 50);

    private final double[] estimatedposes = {0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100};
    private int i = 0;

    public hoodCommands() {
        
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
