package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OI;
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {
    private final DriveSubsystem m_drive = new DriveSubsystem();
    private final CommandXboxController m_controller = 
        new CommandXboxController(Constants.OI.DRIVER_CONTROLLER_PORT);

    private boolean fieldCentric = true;

    public RobotContainer() {
        // Default Command: Left Stick = Move, Right Stick = Turn
        m_drive.setDefaultCommand(
            new DriveCommand(
                m_drive,
                () -> -modifyAxis(-m_controller.getLeftY()), // Forward/Back
                () -> -modifyAxis(m_controller.getLeftX()), // Left/Right
                () -> -modifyAxis(m_controller.getRightX()),// Rotation
                () -> fieldCentric
            )
        );

        configureBindings();
    }

    private void configureBindings() {
        // A Button toggles Field Centric
        m_controller.a().onTrue(new InstantCommand(() -> {
            fieldCentric = !fieldCentric;
            SmartDashboard.putBoolean("Field Centric Enabled", fieldCentric);
        }));

        // Start Button resets Gyro
        m_controller.start().onTrue(new InstantCommand(m_drive::zeroHeading, m_drive));
    }

    public double modifyAxis(double value) {
        // Apply deadband and then cube the result for better control "feel"
        double deadbanded = MathUtil.applyDeadband(value, OI.DEADBAND);
        return Math.pow(deadbanded, 3);
    }
    /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Currently returns null, so the robot will do nothing during autonomous.
    // You can replace this with a path-following command later!
    return null; 
  }
}
