package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class FireFuelCommand extends Command {
    private final ShooterSubsystem m_shooter;
    private final double m_distance = 10.0; // Test distance in feet

    public FireFuelCommand(ShooterSubsystem shooter) {
        this.m_shooter = shooter;
        // This line tells the robot: "While I am running, no one else can use the shooter"
        addRequirements(m_shooter);
    }

    @Override
    public void initialize() {
        // Calls the new 2-motor logic we wrote in the subsystem
        m_shooter.updateShooterTargets(m_distance);
    }

    @Override
    public void execute() {
        // You can check m_shooter.isReady() here to rumble a controller
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the flywheel when the button is released
        m_shooter.stop();
    }

    @Override
    public boolean isFinished() {
        // Command stays active as long as button is held
        return false;
    }
}