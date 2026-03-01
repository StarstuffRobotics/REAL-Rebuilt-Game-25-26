package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.SwerveSubsystem;

public class Robot extends TimedRobot {
  private final XboxController m_driverController =
      new XboxController(Constants.OIConstants.kDriverControllerPort);
  private final SwerveSubsystem m_swerve = new SwerveSubsystem();

  @Override
  public void teleopPeriodic() {
    double xInput =
        -MathUtil.applyDeadband(
            m_driverController.getLeftY(), Constants.OIConstants.kDriveDeadband);
    double yInput =
        -MathUtil.applyDeadband(
            m_driverController.getLeftX(), Constants.OIConstants.kDriveDeadband);
    double rotInput =
        -MathUtil.applyDeadband(
            m_driverController.getRightX(), Constants.OIConstants.kDriveDeadband);

    if (m_driverController.getStartButtonPressed()) {
      m_swerve.zeroGyro();
    }

    if (m_driverController.getXButton()) {
      m_swerve.lockModules();
      return;
    }

    boolean fieldRelative = !m_driverController.getRightBumper();
    m_swerve.drive(xInput, yInput, rotInput, fieldRelative);
  }

  @Override
  public void disabledPeriodic() {
    m_swerve.stop();
  }
}
