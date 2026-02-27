package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import static edu.wpi.first.units.Units.*; 
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OI;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;
import frc.robot.FuelSim;
import frc.robot.Constants.Dimensions; 
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.ExtendedTranslation3d;




public class RobotContainer {
    private final DriveSubsystem m_drive = new DriveSubsystem();
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    private final IntakeSubsystem m_intake = new IntakeSubsystem();
    private final TurretSubsystem m_turret = new TurretSubsystem();
  private int fuelStored;

    private final CommandXboxController m_controller = 
        new CommandXboxController(Constants.OI.DRIVER_CONTROLLER_PORT);

    private boolean fieldCentric = true;
    //new RunCommand(
        // () -> m_robotDrive.drive(
        //     -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
        //     -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
        //     -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
        //     true),
        // m_robotDrive));
    public RobotContainer() {
        m_drive.setDefaultCommand(
            new RunCommand(
            () -> m_drive.drive(
                -MathUtil.applyDeadband(m_controller.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_controller.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_controller.getRightX(), OIConstants.kDriveDeadband),
                true),
            m_drive)
            );
        
//
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
        // Deadband is still good to keep drift away
        if (Math.abs(value) < OI.DEADBAND) return 0;
        
        // Remove the squaring line: return Math.copySign(value * value, value);
        // Return raw linear value instead:
        return value;
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






        private void configureFuelSim() {
            FuelSim instance = FuelSim.getInstance();
            instance.spawnStartingFuel();
            instance.registerRobot(
                Dimensions.FULL_WIDTH,
                Dimensions.FULL_LENGTH,
                Dimensions.BUMPER_HEIGHT,
                m_robotDrive::getPose,
                m_robotDrive::getFieldSpeeds);
            instance.registerIntake(
                -Dimensions.div(Dimensions.FULL_LENGTH, 2), // Divide FULL_LENGTH by 2
                Dimensions.div(Dimensions.FULL_LENGTH, 2), // Divide FULL_LENGTH by 2
                -Dimensions.div(Dimensions.FULL_WIDTH, 2) + Dimensions.Inches.of(7), // Add 7 inches to half the width
                -Dimensions.div(Dimensions.FULL_WIDTH, 2), // Divide FULL_WIDTH by 2
                () -> m_intake.isFrountDeployed() && m_turret.simAbleToIntake(),
                m_turret::simIntake
            );
            instance.registerIntake(
                -Dimensions.div(Dimensions.FULL_LENGTH, 2), // Divide FULL_LENGTH by 2
                Dimensions.div(Dimensions.FULL_LENGTH, 2), // Divide FULL_LENGTH by 2
                Dimensions.div(Dimensions.FULL_WIDTH, 2),  // Divide FULL_WIDTH by 2
                Dimensions.div(Dimensions.FULL_WIDTH, 2) + Dimensions.Inches.of(7), // Add 7 inches to half the width
                () -> m_intake.isFrountDeployed() && m_turret.simAbleToIntake(),
                m_turret::simIntake
            );
            instance.start();
            SmartDashboard.putData(Commands.runOnce(() -> {
                        FuelSim.getInstance().clearFuel();
                        FuelSim.getInstance().spawnStartingFuel();
                    })
                    .withName("Reset Fuel")
                    .ignoringDisable(true));
        }
    
        public ExtendedTranslation3d lanuchVel(LinearVelocity velocity, Angle angle) {
            // Convert the velocity and angle into a 3D velocity vector
            double speed = velocity.in(MetersPerSecond);; // Assuming LinearVelocity has a method to get the value in meters per second
            double angleRadians = angle.in(Radians); // Convert angle to radians using the appropriate method

            // Calculate the velocity components
            double vx = speed * Math.cos(angleRadians); // Horizontal velocity (X-axis)
            double vz = speed * Math.sin(angleRadians); // Vertical velocity (Z-axis)

            // Return the velocity as a Translation3d (Y-axis is 0 since it's not used here)
            return new ExtendedTranslation3d(vx, 0, vz);
        }


        public void launchFuel(LinearVelocity vel, Angle angle) {
            if (fuelStored == 0) return;
            fuelStored--;
            Pose2d pose2d = m_robotDrive.getPose();

            Pose3d robot = new Pose3d(
                pose2d.getX(),
                pose2d.getY(),
                0.5, // Assume the fuel is launched from a height of 0.5 meters
                new Rotation3d(0, 0, pose2d.getRotation().getRadians())
            );

            ExtendedTranslation3d initialPosition = new ExtendedTranslation3d(robot.getTranslation());
            FuelSim.getInstance().spawnFuel(initialPosition, lanuchVel(vel, angle));
        }
}