package frc.robot.commands.turret;

import frc.robot.subsystems.turret.shooterSubsystem;
import frc.robot.subsystems.turret.shooterSubsystem;

public class shooterCommands {
    shooterSubsystem shooter;

    public shooterCommands(shooterSubsystem shooter){
        this.shooter = shooter;
    }

    public void shootTurretOut(){
        shooter.startMotor(1);//
    }

    public void shootTurretIn(){
        shooter.startMotor(-1);
    }
}
