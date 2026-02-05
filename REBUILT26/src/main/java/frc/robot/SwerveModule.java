package frc.robot;

// SwerveModule.java
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {

  private final CANSparkFlex driveMotor;
  private final CANSparkFlex angleMotor;

  public SwerveModule(int driveID, int angleID) {
    driveMotor = new CANSparkFlex(driveID, MotorType.kBrushless);
    angleMotor = new CANSparkFlex(angleID, MotorType.kBrushless);

    driveMotor.restoreFactoryDefaults();
    angleMotor.restoreFactoryDefaults();
  }

  public Rotation2d getAngle() {
    // Placeholder – replace with absolute encoder
    return Rotation2d.fromRadians(
        angleMotor.getEncoder().getPosition()
    );
  }

  public void setState(SwerveModuleState state) {
    SwerveModuleState optimized =
        SwerveModuleState.optimize(state, getAngle());

    driveMotor.set(optimized.speedMetersPerSecond / Constants.Swerve.MAX_SPEED);
    angleMotor.set(optimized.angle.getRadians() / Math.PI);
  }
}
