package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OI;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class RobotContainer {
    private final DriveSubsystem m_drive = new DriveSubsystem();

    private final CommandXboxController m_controller =
        new CommandXboxController(OI.DRIVER_CONTROLLER_PORT);

    public RobotContainer() {
        m_drive.setDefaultCommand(
            new RunCommand(
            () -> {
                double forward = -MathUtil.applyDeadband(m_controller.getLeftY(), OI.DEADBAND);

                if (forward == 0) {
                    m_drive.stop();
                } else {
                    m_drive.drive(forward);
                }
            },
            m_drive)
        );

        configureBindings();
    }

    private void configureBindings() {
    }

    public Command getAutonomousCommand() {
        return null;
    }
}
