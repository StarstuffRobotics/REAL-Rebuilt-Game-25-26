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

    // public void findOptimalHoodAngle(){
    //     hood.findOptimalHoodAngle(rotation.getDistanceToTarget());
    // }
    
    public void hoodUp(){
        hood.setHoodSpeed();
    }
    
    public void hoodDown(){
        hood.setHoodReversedSpeed();
    }

    public void runTurret(){
        allignTurret();
        //findOptimalHoodAngle();
        shootTurret();

    }

    public void manualTurretRight(){
        rotation.manualRotateRight();
    }

    public void manualTurretLeft(){
        rotation.manualRotateLeft();
    }

    public void shootTurret(){
        shooter.startMotor();
    }

    public void shootTurretSpeed(){
        shooter.startMotorSpeed();
    }

    public void shooterReverse(){
        shooter.shooterReverse();
    }

    public void stopRotation(){
        rotation.stopRotation();
    }

  

    public void shooterStop(){
        shooter.shooterStop();
    }

    public void stopTurret(){
        stopRotation();
        //stopHood();
        shooterStop();
    }
}
