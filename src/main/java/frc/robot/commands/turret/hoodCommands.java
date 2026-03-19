package frc.robot.commands.turret;
import frc.robot.subsystems.turret.hoodSubsystem;


public class hoodCommands  {
    
    private final hoodSubsystem m_hoodSubsystem;


    
    public hoodCommands(hoodSubsystem hoodSubsystem) {
        m_hoodSubsystem = hoodSubsystem;
    }

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
