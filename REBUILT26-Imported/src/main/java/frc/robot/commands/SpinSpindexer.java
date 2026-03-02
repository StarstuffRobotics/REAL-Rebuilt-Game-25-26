package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Spindexer;

public class SpinSpindexer extends Command {
    private final Spindexer m_spindexer;

    public SpinSpindexer(Spindexer subsystem) {
        m_spindexer = subsystem;
        // This ensures no other command uses the spindexer at the same time
        addRequirements(m_spindexer);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_spindexer.spin();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_spindexer.stop();
    }
}