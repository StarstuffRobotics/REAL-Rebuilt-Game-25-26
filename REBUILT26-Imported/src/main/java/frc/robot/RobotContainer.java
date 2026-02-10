package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OI;

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
                () -> -modifyAxis(m_controller.getLeftY()),   // Forward/Backward
                () -> -modifyAxis(m_controller.getLeftX()),   // Left/Right strafe
                () -> -modifyAxis(m_controller.getRightX()),  // Rotation
                () -> fieldCentric
            )
        );

        configureBindings();
    }

    private void configureBindings() {
        // Toggle Field Centric mode when 'A' is pressed
        m_controller.a().onTrue(new InstantCommand(() -> {
            fieldCentric = !fieldCentric;
            SmartDashboard.putBoolean("Field Centric Enabled", fieldCentric);
        }));

        // Reset Gyro when 'Start' button is pressed
        // (Useful if the robot's "Forward" gets drifted)
        m_controller.start().onTrue(m_drive.runOnce(m_drive::zeroHeading));
    }

    /**
     * Applies a deadband and squares the input for smoother control.
     * @param value Raw joystick input
     * @return Processed input
     */
    private double modifyAxis(double value) {
        // Deadband
        if (Math.abs(value) < OI.DEADBAND) return 0;

        // Square the input (value^2) to make fine movements easier
        // Math.copySign ensures that -0.5 becomes -0.25 instead of +0.25
        return Math.copySign(value * value, value);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     * Currently returns null (no autonomous).
     */
    public Command getAutonomousCommand() {
        // When you're ready for auto, you can return a command here like:
        // return new PathPlannerCommand(...);
        return null;
    }
}