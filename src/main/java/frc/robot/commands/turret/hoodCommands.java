package frc.robot.commands.turret;
import frc.robot.subsystems.turret.HoodSubsystem;


public class hoodCommands  {
    
    private final HoodSubsystem m_hoodSubsystem;


    
    public hoodCommands(HoodSubsystem hoodSubsystem) {
        m_hoodSubsystem = hoodSubsystem;
    }

    // public void setHoodSpeed(double speed) {
    //     m_hoodSubsystem.setSpeed(speed);
    // }

    // public void setHoodSpeed() {
    //     m_hoodSubsystem.setSpeed();
    // }

    // public void setHoodReversedSpeed(double speed) {
    //     m_hoodSubsystem.setReversedSpeed(speed);
    // }

    // public void setHoodReversedSpeed() {
    //     m_hoodSubsystem.setReversedSpeed();
    // }

     public void getHoodAngle() {
        m_hoodSubsystem.getHoodAngle();
    }

    public void setHoodZero() {
        m_hoodSubsystem.setHoodZero();
    }

    public void setHoodMax() {
        m_hoodSubsystem.setHoodMax();
    }

  
    
}
