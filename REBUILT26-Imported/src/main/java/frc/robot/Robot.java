package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to run this class automatically. 
 * This is the "brain" that connects your code to the hardware.
 */
public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer;

    /**
     * This function is run when the robot is first started up.
     * We initialize the RobotContainer here, which sets up our Subsystems and Bindings.
     */
    @Override
    public void robotInit() {
        // Instantiate our RobotContainer. 
        // This will perform all our button bindings and subsystem initialization.
        m_robotContainer = new RobotContainer();
    }

    /**
     * This function is called every 20 ms, no matter the mode.
     * This is the most important part for Command-Based programming.
     */
    @Override
    public void robotPeriodic() {
        // Must be called to allow the CommandScheduler to run commands, 
        // poll buttons, and update subsystem periodic methods.
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    /** This autonomous runs the autonomous command selected by your RobotContainer class. */
    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when teleop starts.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        // We leave this empty because the CommandScheduler 
        // is handling our DefaultDrive command!
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void simulationInit() {}

    @Override
    public void simulationPeriodic() {}
}