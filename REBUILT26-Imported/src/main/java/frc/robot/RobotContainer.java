package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.*;
import frc.robot.commands.*;

public class RobotContainer {
    // Subsystems
    private final Spindexer m_spindexer = new Spindexer();
    private final Accelerator m_accelerator = new Accelerator();

    private final CommandXboxController m_driverController = new CommandXboxController(0);

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Spindexer on 'A'
        m_driverController.a().toggleOnTrue(new SpinSpindexer(m_spindexer));

        // Accelerator on 'X'
        m_driverController.x().toggleOnTrue(new SpinAccelerator(m_accelerator));
    }
}