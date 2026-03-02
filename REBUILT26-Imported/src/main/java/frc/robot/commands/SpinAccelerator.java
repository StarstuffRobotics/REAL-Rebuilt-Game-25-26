package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Accelerator;

public class SpinAccelerator extends Command {
    private final Accelerator m_accel;

    public SpinAccelerator(Accelerator subsystem) {
        m_accel = subsystem;
        addRequirements(m_accel);
    }

    @Override
    public void initialize() {
        m_accel.spin();
    }

    @Override
    public void end(boolean interrupted) {
        m_accel.stop();
    }
}