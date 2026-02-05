package frc.robot;

// DriveCommand.java
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveCommand extends Command {

  private final DriveSubsystem drive;
  private final DoubleSupplier x, y, rot;

  public DriveCommand(
      DriveSubsystem drive,
      DoubleSupplier x,
      DoubleSupplier y,
      DoubleSupplier rot) {

    this.drive = drive;
    this.x = x;
    this.y = y;
    this.rot = rot;
    addRequirements(drive);
  }

  @Override
  public void execute() {
    drive.drive(
        x.getAsDouble(),
        y.getAsDouble(),
        rot.getAsDouble()
    );
  }
}
