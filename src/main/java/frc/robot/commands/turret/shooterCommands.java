package frc.robot.commands.turret;

import frc.robot.subsystems.turret.shooterSubsystem;

public class shooterCommands {
    shooterSubsystem shooter;

    public shooterCommands(shooterSubsystem shooter){
        this.shooter = shooter;
    }

    public void startMotor(double speed){
        shooter.shooterOnOff(speed);
    }

    public void shooterReverse(double speed){
        shooter.shooterReverse(speed);
    }

    public void shooterStop() {
        shooter.shooterStop();
    }

    public void stopTurret(){
        shooter.shooterStop();
    }
}
