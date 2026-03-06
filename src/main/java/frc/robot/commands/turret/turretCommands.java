package frc.robot.commands.turret;

import frc.robot.subsystems.turret.rotationSubsystem;

public class turretCommands {
    shooterCommands shooter;
    rotationSubsystem rotation;
    hoodCommands hood;

    public turretCommands(shooterCommands shooter,rotationSubsystem rotation, hoodCommands hood){
        this.shooter = shooter;
        this.rotation = rotation;   
        this.hood = hood;
    }

    public void allignTurret(){
        rotation.rotateTurret();
    }

    public void findOptimalHoodAngle(){
        hood.findOptimalHoodAngle(rotation.getDistanceToTarget());
    }
    
    public void runTurret(){
        allignTurret();
        findOptimalHoodAngle();
    }

    public void stopRotation(){
        rotation.stopRotation();
    }

    public void setHoodAngleZero(){
        hood.setHoodAngleCustom(0);
    }
    
    public void setHoodAngle(double hoodAngle){
        hood.setHoodAngleCustom(hoodAngle);
    }

}
