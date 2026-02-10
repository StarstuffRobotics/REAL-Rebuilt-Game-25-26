package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotContainer {
    private final DriveSubsystem m_drive = new DriveSubsystem();
    private final CommandXboxController m_controller = 
        new CommandXboxController(Constants.OI.DRIVER_CONTROLLER_PORT);

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
        m_controller.a().onTrue(new InstantCommand(() -> {
            fieldCentric = !fieldCentric;
            SmartDashboard.putBoolean("Field Centric Enabled", fieldCentric);
        }));

        // Reset Heading
        m_controller.start().onTrue(new InstantCommand(m_drive::zeroHeading, m_drive));
    }

    private double modifyAxis(double value) {
        // Use the full path for Constants to ensure the IDE resolves it correctly
        if (Math.abs(value) < Constants.OI.DEADBAND) return 0.0;
        return Math.copySign(value * value, value);
    }

    public Command getAutonomousCommand() {
        return null;
    }
}