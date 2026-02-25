package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.Swerve;
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {
    private final DriveSubsystem m_drive = new DriveSubsystem();
    
    private final CommandXboxController m_controller = 
        new CommandXboxController(0);

    private boolean fieldCentric = true;

    public RobotContainer() {
        // Correctly instantiating the DriveCommand class with 5 arguments
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

        m_controller.start().onTrue(new InstantCommand(m_drive::zeroHeading, m_drive));
    }

    private double modifyAxis(double value) {
        return MathUtil.applyDeadband(value, Swerve.DEADBAND);
    }

    public Command getAutonomousCommand() {
        return null; 
    }
}