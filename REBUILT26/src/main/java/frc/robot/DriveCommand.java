package frc.robot;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCommand extends Command {
    private final DriveSubsystem m_drive;
    private final DoubleSupplier m_xSpeed;
    private final DoubleSupplier m_ySpeed;
    private final DoubleSupplier m_rot;
    private final boolean m_fieldRelative;

    public DriveCommand(
            DriveSubsystem subsystem,
            DoubleSupplier xSpeed,
            DoubleSupplier ySpeed,
            DoubleSupplier rot,
            boolean fieldRelative) {
        
        this.m_drive = subsystem;
        this.m_xSpeed = xSpeed;
        this.m_ySpeed = ySpeed;
        this.m_rot = rot;
        this.m_fieldRelative = fieldRelative;

        addRequirements(m_drive);
    }

    @Override
    public void execute() {
        // Apply a 10% deadband to prevent stick drift
        double x = MathUtil.applyDeadband(m_xSpeed.getAsDouble(), 0.1);
        double y = MathUtil.applyDeadband(m_ySpeed.getAsDouble(), 0.1);
        double r = MathUtil.applyDeadband(m_rot.getAsDouble(), 0.1);

        // Send to subsystem (Max speed 4.5 m/s, Max rotation 2.0 rad/s)
        m_drive.drive(x * 4.5, y * 4.5, r * 2.0, m_fieldRelative);
    }
}