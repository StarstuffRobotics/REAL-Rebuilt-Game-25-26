package frc.robot.commands.turret;
import frc.robot.subsystems.turret.hoodSubsystem;


public class hoodCommands {
    hoodSubsystem hood;

    public double distance = 0 ;
    public hoodCommands(hoodSubsystem hood){
        this.hood = hood;
    }

    public void findOptimalHoodAngle(double distanceFeet){//will find angle then set angle pass in the distence form the lime light
        hood.findOptomalHoodAngle(distanceFeet);
    }

    public void setHoodAngleCustom(double hoodAngle){
        hood.setHoodAngleCustom(hoodAngle);
    }

    public void stopHood(){
        hood.stopHood();
    }

    public double getHoodAngle(){
        return hood.getHoodAngle();
    }

    public void cycleHoodAngleForward(){
        hood.cycleHoodAngle(1);
    }

    public void cycleHoodAngleBackward(){
        hood.cycleHoodAngle(-1);
    }
    
}
