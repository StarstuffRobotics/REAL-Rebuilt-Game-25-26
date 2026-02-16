package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OI;

public class RobotContainer {
    private final DriveSubsystem m_drive = new DriveSubsystem();
    private final CommandXboxController m_controller = 
        new CommandXboxController(Constants.OI.DRIVER_CONTROLLER_PORT);

    private boolean fieldCentric = true;

    public RobotContainer() {
        m_drive.setDefaultCommand(
            new DriveCommand(
                m_drive,
                () -> -modifyAxis(m_controller.getLeftY()), // This MUST be X (Forward/Back)
                () -> -modifyAxis(m_controller.getLeftX()), // This MUST be Y (Left/Right)
                () -> -modifyAxis(m_controller.getRightX()),// This MUST be Rotation
                () -> fieldCentric,
                this
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

    public double modifyAxis(double value) {
        if (Math.abs(value) < OI.DEADBAND) return 0;
        return Math.copySign(value * value, value);
    }

    public double getDriveForward(){
        return m_controller.getLeftY();
    }

    public double getDriveTurn(){
        return m_controller.getRightX();
    }

    public Command getAutonomousCommand() {

        // Replace with the actual autonomous command

        return null; // Return your autonomous command here

    }
}