package frc.robot.commands.turret;

import frc.robot.subsystems.turret.rotationSubsystem;


public class rotationCommand {
    private rotationSubsystem turret;

    public rotationCommand(rotationSubsystem turret){
        this.turret = turret;
    }

    public boolean getHasTarget(){
        return turret.hasTarget();
    }

 
    
}
