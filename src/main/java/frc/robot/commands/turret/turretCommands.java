package frc.robot.commands.turret;

import frc.robot.subsystems.turret.hoodSubsystem;
import frc.robot.subsystems.turret.rotationSubsystem;
import frc.robot.subsystems.turret.shooterSubsystem;
public class turretCommands {
    shooterSubsystem shooter;
    rotationSubsystem rotation;
    hoodSubsystem hood;

    public turretCommands(shooterSubsystem shooter,rotationSubsystem rotation, hoodSubsystem hood){
        this.shooter = shooter;
        this.rotation = rotation;   
        this.hood = hood;
    }

    public void allignTurret(){

    }

    public void shootTurretOut(){

    }

    public void shootTurretIn(){
        
    }
    
}
