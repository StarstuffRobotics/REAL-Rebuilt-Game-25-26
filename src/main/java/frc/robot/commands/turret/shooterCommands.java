package frc.robot.commands.turret;

import frc.robot.subsystems.turret.shooterSubsystem;

public class shooterCommands {
    shooterSubsystem shooter;

    public shooterCommands(shooterSubsystem shooter){
        this.shooter = shooter;
    }

    public void shootTurretOut(){
        shooter.startMotor();//
    }

    public void shootTurretIn(){
        shooter.startMotor();
    }

    public void startMotor(double speed){
        shooter.startMotor(speed);
    }

    public void stopTurret(){
        shooter.stopMotor();
    }
}
