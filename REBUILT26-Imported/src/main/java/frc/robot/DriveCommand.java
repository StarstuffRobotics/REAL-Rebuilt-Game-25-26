package frc.robot;

import java.util.function.DoubleSupplier;
import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCommand extends Command {
    private final DriveSubsystem drive;
    private final DoubleSupplier x, y, rot;
    private final BooleanSupplier isFieldRelative;

    // Constructor with exactly 5 parameters to match RobotContainer
    public DriveCommand(
        DriveSubsystem drive, 
        DoubleSupplier x, 
        DoubleSupplier y, 
        DoubleSupplier rot, 
        BooleanSupplier isFieldRelative) {
        
        this.drive = drive;
        this.x = x;
        this.y = y;
        this.rot = rot;
        this.isFieldRelative = isFieldRelative;

        addRequirements(drive);
    }

    @Override
    public void execute() {
        drive.drive(
            x.getAsDouble() * Constants.Swerve.MAX_SPEED,
            y.getAsDouble() * Constants.Swerve.MAX_SPEED,
            rot.getAsDouble() * Constants.Swerve.MAX_ANGULAR_SPEED,
            isFieldRelative.getAsBoolean()
        );
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(0, 0, 0, false);
    }
}