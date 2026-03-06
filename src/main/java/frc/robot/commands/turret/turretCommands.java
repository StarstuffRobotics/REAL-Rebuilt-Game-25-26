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
    
    public void hoodUp(){
        hood.setHoodAngleCustom(hood.getHoodAngle() + 0.1);
    }
    
    public void hoodDown(){
        hood.setHoodAngleCustom(hood.getHoodAngle() - 0.1);
    }

    public void runTurret(){
        allignTurret();
        //findOptimalHoodAngle();
        shootTurret();

    }

    public void shootTurret(){
        shooter.startMotor();
    }

    public void shooterReverse(){
        shooter.shooterReverse();
    }

    public void stopRotation(){
        rotation.stopRotation();
    }

    public void stopHood(){
        hood.stopHood();
    }

    public void shooterStop(){
        shooter.shooterStop();
    }

    public void stopTurret(){
        stopRotation();
        stopHood();
        shooterStop();
    }

    public void setHoodAngleZero(){
        hood.setHoodAngleCustom(0);
    }
    
    public void setHoodAngle(double hoodAngle){
        hood.setHoodAngleCustom(hoodAngle);
    }

}
