// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//
package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SpindexerConstants;
import frc.robot.commands.Intake.intakeCommands;
import frc.robot.commands.accelerator.acceleratorCommands;
import frc.robot.commands.spindexer.spindexerCommand;
import frc.robot.commands.turret.hoodCommands;
import frc.robot.commands.turret.shooterCommands;
import frc.robot.commands.turret.turretCommands;
import frc.robot.subsystems.accelerator.acceleratorSubsystem;
import frc.robot.subsystems.intake.intakeSubsystem;
import frc.robot.subsystems.spindexer.spindexerSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.turret.hoodSubsystem; // Ensure this is the correct package for shooterCommands
import frc.robot.subsystems.turret.rotationSubsystem; // Ensure this is the correct package for rotationCommands
import frc.robot.subsystems.turret.shooterSubsystem; // Ensure this is the correct package for hoodCommands
import swervelib.SwerveInputStream;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final         CommandXboxController driverXbox = new CommandXboxController(0);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/neo"));
  // Intake
  private final intakeSubsystem intakes = new intakeSubsystem();
  private final intakeCommands intake = new intakeCommands(intakes);

  // Accelerator
  private final acceleratorSubsystem acceleratorSubsystem = new acceleratorSubsystem();
  private final acceleratorCommands acceleratorCommands = new acceleratorCommands(acceleratorSubsystem);

  // Spindexer
  private final spindexerSubsystem spindexer   = new spindexerSubsystem();
  private final spindexerCommand spindexerCommand = new spindexerCommand(spindexer);

  private final shooterSubsystem shooterSubsystem = new shooterSubsystem();
  private final shooterCommands shooter = new shooterCommands(shooterSubsystem);
  
  private final rotationSubsystem rotation = new rotationSubsystem();
  
  private final hoodSubsystem hoodSubsystem = new hoodSubsystem();
  private final hoodCommands hood = new hoodCommands(hoodSubsystem);
  
  private final turretCommands turret = new turretCommands(shooter, rotation, hood);
  
  private boolean alleianceRelativeControlDefault = true;
  
  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY(), //so that forward on the joystick is forward on the field, it was reversed
                                                                () -> driverXbox.getLeftX()) //so that the right stick rotates, will add strafing soon. I do not know if I can do this tho bc. there is that thing on line 57.
                                                              .withControllerRotationAxis(() -> driverXbox.getRightX() * -1) // Rotation
                                                              .deadband(OperatorConstants.DEADBAND)
                                                              .scaleTranslation(0.8)
                                                              .allianceRelativeControl(false);


  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
                                                                                             driverXbox::getRightY)
                                                           .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                        () -> -driverXbox.getLeftY(),
                                                                        () -> -driverXbox.getLeftX())
                                                                    .withControllerRotationAxis(() -> driverXbox.getRawAxis(
                                                                        2))
                                                                    .deadband(OperatorConstants.DEADBAND)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
                                                                               .withControllerHeadingAxis(() ->
                                                                                                              Math.sin(
                                                                                                                  driverXbox.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2),
                                                                                                          () ->
                                                                                                              Math.cos(
                                                                                                                  driverXbox.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2))
                                                                               .headingWhile(true)
                                                                               .translationHeadingOffset(true)
                                                                               .translationHeadingOffset(Rotation2d.fromDegrees(
                                                                                   0));

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {

    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("shootTurret",Commands.runOnce(()-> turret.shootTurretSpeed()));
    NamedCommands.registerCommand("Accelator", Commands.runOnce(acceleratorCommands::spinToggle));
    NamedCommands.registerCommand("SpindexerOn", Commands.runOnce(()-> spindexer.spin(SpindexerConstants.kSpindexerSpeed)));
    NamedCommands.registerCommand("setX",Commands.runOnce(drivebase::lock));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen                   = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngleKeyboard);
    
    
    
    if (RobotBase.isSimulation())
    {
      //drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    } else
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    if (Robot.isSimulation())
    {
      Pose2d target = new Pose2d(new Translation2d(1, 4),
                                 Rotation2d.fromDegrees(90));
      //drivebase.getSwerveDrive().field.getObject("targetPose").setPose(target);
      driveDirectAngleKeyboard.driveToPose(() -> target,
                                           new ProfiledPIDController(5,
                                                                     0,
                                                                     0,
                                                                     new Constraints(5, 2)),
                                           new ProfiledPIDController(5,
                                                                     0,
                                                                     0,
                                                                     new Constraints(Units.degreesToRadians(360),
                                                                                     Units.degreesToRadians(180))
                                           ));
      driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
      driverXbox.button(2).whileTrue(Commands.runEnd(() -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
                                                     () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));

//      driverXbox.b().whileTrue(
//          drivebase.driveToPose(
//              new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
//                              );

    }
      if (DriverStation.isTestEnabled())
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      //driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.y().onTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      driverXbox.leftBumper().onTrue(Commands.none());
      driverXbox.rightBumper().onTrue(Commands.none());
    } else
    {

      //so this is telop, I think...

      // planed button bindings, subject to change:
      // a while true: everything in reverse exept shoorter 
      // b on true: intake roller toggle (up down)
      // y on true: shooter, accelerator, spindexer the right way toggle on off
      // x on true: intake wheels toggle (in out) 
      // d pad up while true: turret hood up
      // d pad down while true: turret hood down
      // d pad left on true: hang down
      // d pad right on true: hang up
      // left bumper on true: slow mode toggle (fast,slow)
      // right bumper while true: set X
      // right trigger while true: adjust turret to the right (override the limelight)
      // left trigger while true: adjust turret to the left (override the limelight)
      // left stickY while true: forwards and backwards
      // left stickX while true: strafing left right
      // right stickX while true: rotation


      
      // driverXbox.a().onTrue(Commands.runOnce(() -> {
      //     if (intakeSubsystem.getIsUp()) {
      //       intakeCommands.intakeDown(10.0);
      //     } else {
      //       intakeCommands.intakeUp(10.0);
      //     }
      //   }));
      
      // Intake
      driverXbox.b().onTrue(Commands.runOnce(()-> intake.intakeUpDown()));
      
      driverXbox.x().onTrue(Commands.runOnce(()-> intake.rollerInOff()));
      
      driverXbox.a().onTrue(Commands.runOnce(()-> intake.rollerOut()));
      driverXbox.a().onFalse(Commands.runOnce(()-> intake.rollerStop()));
      
      driverXbox.b().onFalse(Commands.runOnce(()-> intake.intakeStop()));
     

      // Spindexer
      driverXbox.a().onTrue((Commands.runOnce(spindexerCommand::reversedSpin)));
      driverXbox.a().onFalse(Commands.runOnce(spindexerCommand::stop));
      
      driverXbox.y().onTrue(Commands.runOnce(()-> spindexer.spin(SpindexerConstants.kSpindexerSpeed)));

      // Accelerator
      driverXbox.y().onTrue(Commands.runOnce(acceleratorCommands::spinToggle));//off on
      driverXbox.a().onTrue(Commands.runOnce(acceleratorCommands::reverseSpin));//off on but reverse
      driverXbox.a().onFalse(Commands.runOnce(acceleratorCommands::stop));

      // Turret
      driverXbox.y().onTrue(Commands.runOnce(() -> turret.shootTurretSpeed()));
      
      driverXbox.a().onTrue(Commands.runOnce(()-> turret.shooterReverse()));
      driverXbox.a().onFalse(Commands.runOnce(()-> turret.shooterStop()));

      driverXbox.leftTrigger().onTrue(Commands.runOnce(()-> turret.manualTurretRight()));//rotate the turret left manualy
      driverXbox.rightTrigger().onTrue(Commands.runOnce(()-> turret.manualTurretLeft()));//rotate the turret right manualy
      driverXbox.rightTrigger().onFalse(Commands.runOnce(()-> turret.stopRotation()));
      driverXbox.leftTrigger().onFalse(Commands.runOnce(()-> turret.stopRotation()));
      
      driverXbox.povUp().onTrue(Commands.runOnce(()-> turret.cycleHoodAngleForward()));//hood up
      driverXbox.povDown().onTrue(Commands.runOnce( () -> turret.cycleHoodAngleBackward()));//hood down
      //driverXbox.y().onFalse(Commands.runOnce(()-> turret.stopRotation()));

      // Other Stuff
      ////driverXbox.a().whileTrue(Commands.runOnce(drivebase::addFakeVisionReading)); // what does this do?
      driverXbox.start().whileTrue(Commands.none());
      driverXbox.back().whileTrue(Commands.none());
      driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      // riverXbox.rightBumper().onTrue(Commands.runOnce(() -> {
      //   alleianceRelativeControlDefault = !alleianceRelativeControlDefault;
      // }));//toggle alliance-centric control (alliance relative control is on by default, so this would turn it off and on)
    }

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("New Auto");
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
