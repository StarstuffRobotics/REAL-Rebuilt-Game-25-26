package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
    private final DriveSubsystem m_drive = new DriveSubsystem();
    private final CommandXboxController m_controller = new CommandXboxController(Constants.OI.DRIVER_CONTROLLER_PORT);
    private boolean fieldCentric = true;

    public RobotContainer() {
        m_drive.setDefaultCommand(
            new DriveCommand(
                m_drive,
                () -> -modifyAxis(m_controller.getLeftY()),
                () -> -modifyAxis(m_controller.getLeftX()),
                () -> -modifyAxis(m_controller.getRightX()),
                () -> fieldCentric
            )
        );
        configureBindings();
    }

    private void configureBindings() {
        m_controller.a().onTrue(new InstantCommand(() -> fieldCentric = !fieldCentric));
        m_controller.start().onTrue(m_drive.runOnce(m_drive::zeroHeading));
    }

    private double modifyAxis(double value) {
        if (Math.abs(value) < Constants.OI.DEADBAND) return 0;
        return Math.copySign(value * value, value);
    }
}