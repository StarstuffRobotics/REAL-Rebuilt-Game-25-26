package frc.robot;

import java.util.function.DoubleSupplier;
import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveCommand extends Command {
    private final DriveSubsystem drive;
    private final DoubleSupplier x, y, rot;
    private final BooleanSupplier isFieldRelative;

    public DriveCommand(DriveSubsystem drive, DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot, BooleanSupplier isFieldRelative) {
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
}