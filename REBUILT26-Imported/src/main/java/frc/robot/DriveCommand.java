package frc.robot;

import java.util.function.DoubleSupplier;
import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.Command;



public class DriveCommand extends Command {
    private final DriveSubsystem drive;
    private final DoubleSupplier x, y, rot;
    private final BooleanSupplier isFieldRelative;
    private final RobotContainer robotContainer;

    public DriveCommand(DriveSubsystem drive, DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot, BooleanSupplier isFieldRelative, RobotContainer robotContainer) {
        this.drive = drive;
        this.x = x;
        this.y = y;
        this.rot = rot;
        this.isFieldRelative = isFieldRelative;
        this.robotContainer = robotContainer;
        // This tells the robot that no other command can use the drive base while this is running
        addRequirements(drive);
    }

    @Override
    public void execute() {
        // Send the real-time joystick values to the subsystem
        double modifiedX = x.getAsDouble();
        double modifiedY = y.getAsDouble();
        double modifiedRot = rot.getAsDouble();

        drive.drive(
            modifiedX * Constants.Swerve.MAX_SPEED,
            modifiedY * Constants.Swerve.MAX_SPEED,
            modifiedRot * Constants.Swerve.MAX_ANGULAR_SPEED,
            isFieldRelative.getAsBoolean()
        );
    }
}