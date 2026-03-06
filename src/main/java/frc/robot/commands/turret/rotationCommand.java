package frc.robot.commands.turret;

import frc.robot.subsystems.turret.rotationSubsystem;

public class rotationCommand {
    private rotationSubsystem turret;
    private double distance;

    public rotationCommand(rotationSubsystem turret){
        this.turret = turret;
    }

    public boolean getHasTarget(){
        return turret.hasTarget();
    }
    
    public double getDistanceToTarget(){
        distance = turret.getDistanceToTarget();

        return distance;
    }

    public void stopRotation(){
        turret.stopRotation();
    }
    
    

    
    
}
