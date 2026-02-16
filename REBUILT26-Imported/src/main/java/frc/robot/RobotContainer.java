package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OI;
import frc.robot.Constants.Swerve;


public class RobotContainer {
    // 1. Subsystems
    private final DriveSubsystem m_drive = new DriveSubsystem();

    // 2. Controllers
    private final CommandXboxController m_controller = 
        new CommandXboxController(OI.DRIVER_CONTROLLER_PORT);

    // 3. State Variables
    private boolean fieldCentric = true;

    public RobotContainer() {
        // Configure the default command for drive (runs automatically)
        // Note: We invert the Y and X because Xbox controllers return negative for "Up"
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

    /**
     * Applies a deadband and squares the input for smoother control.
     * @param value Raw joystick input
     * @return Processed input
     */

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     * Currently returns null (no autonomous).
     */
    public Command getAutonomousCommand() {
        // When you're ready for auto, you can return a command here like:
        // return new PathPlannerCommand(...);
        return null;
    }
    private void configureBindings() {
        m_controller.a().onTrue(new InstantCommand(() -> {
            fieldCentric = !fieldCentric;
            SmartDashboard.putBoolean("Field Centric Enabled", fieldCentric);
        }));
        m_controller.start().onTrue(m_drive.runOnce(m_drive::zeroHeading));
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
}