package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OI;
import frc.robot.Constants.Swerve;

public class RobotContainer {
    private final DriveSubsystem m_drive = new DriveSubsystem();
    private final CommandXboxController m_controller = new CommandXboxController(OI.DRIVER_CONTROLLER_PORT);
    private boolean fieldCentric = true;

    public RobotContainer() {
        m_drive.setDefaultCommand(
            new DriveCommand(
                m_drive,
                // Apply slow mode multiplier if Right Bumper is held
                () -> -modifyAxis(m_controller.getLeftY()) * (m_controller.getHID().getRightBumper() ? Swerve.SLOW_SPEED_FACTOR : 1.0),
                () -> -modifyAxis(m_controller.getLeftX()) * (m_controller.getHID().getRightBumper() ? Swerve.SLOW_SPEED_FACTOR : 1.0),
                () -> -modifyAxis(m_controller.getRightX()) * (m_controller.getHID().getRightBumper() ? Swerve.SLOW_ROTATION_FACTOR : 1.0),
                () -> fieldCentric
            )
        );

        configureBindings();
    }

    private void configureBindings() {
        // A Button: Toggle Field Centric
        m_controller.a().onTrue(new InstantCommand(() -> {
            fieldCentric = !fieldCentric;
            SmartDashboard.putBoolean("Field Centric Enabled", fieldCentric);
        }));

        // Start Button: Reset Gyro
        m_controller.start().onTrue(m_drive.runOnce(m_drive::zeroHeading));
    }

    private double modifyAxis(double value) {
        if (Math.abs(value) < OI.DEADBAND) return 0;
        return Math.copySign(value * value, value);
    }

    public Command getAutonomousCommand() {
        return null;
    }
}