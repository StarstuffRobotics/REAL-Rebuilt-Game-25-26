// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private final Timer matchTimer = new Timer();
  private NetworkTableEntry matchTimeEntry;

  @Override
  public void robotInit() {
    // Set up the command-based container (includes Xbox controller bindings and default drive).
    m_robotContainer = new RobotContainer();

    // Set up NetworkTables entry for match time (for Elastic Dashboard or similar).
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable dashboardTable = inst.getTable("ElasticDashboard");
    matchTimeEntry = dashboardTable.getEntry("MatchTime");
  }

  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, running commands,
    // and calling subsystem periodic methods.
    CommandScheduler.getInstance().run();

    // Push formatted match time to dashboard.
    double matchTime = DriverStation.getMatchTime();
    if (matchTime < 0) {
      matchTime = matchTimer.get();
    }

    double timeRemaining = DriverStation.isAutonomous() ? 15.0 - matchTime : 135.0 - matchTime;
    int minutes = (int) (timeRemaining / 60);
    int seconds = (int) (timeRemaining % 60);
    String timeFormatted = String.format("%d:%02d", minutes, seconds);
    if (matchTimeEntry != null) {
      matchTimeEntry.setString(timeFormatted);
    }
    if (RobotBase.isSimulation()) {
    NetworkTable limelight =
        NetworkTableInstance.getDefault().getTable("limelight");

    limelight.getEntry("tv").setNumber(1);   // target visible
    limelight.getEntry("tx").setNumber(6.0); // target to the right
    limelight.getEntry("tid").setNumber(1);  // AprilTag ID 1
}
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    } else {
      System.out.println("Autonomous command was null, skipping auto.");
    }

    matchTimer.reset();
    matchTimer.start();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // Stop autonomous when teleop begins.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    matchTimer.reset();
    matchTimer.start();
  }

  @Override
  public void teleopPeriodic() {
    // Teleop control handled by default command in RobotContainer using the Xbox controller.
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}
}