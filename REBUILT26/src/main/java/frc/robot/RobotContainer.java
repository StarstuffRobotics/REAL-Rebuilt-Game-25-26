package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  private static final double DEADBAND = 0.08;

  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final CommandXboxController driverController = new CommandXboxController(0);

  public RobotContainer() {
    driveSubsystem.setDefaultCommand(
        new DriveCommand(
            driveSubsystem,
            () -> -MathUtil.applyDeadband(driverController.getLeftY(), DEADBAND),
            () -> -MathUtil.applyDeadband(driverController.getLeftX(), DEADBAND),
            () -> -MathUtil.applyDeadband(driverController.getRightX(), DEADBAND)));
  }

  public Command getAutonomousCommand() {
    return Commands.none();
  }
}
