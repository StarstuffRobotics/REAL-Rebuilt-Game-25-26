package frc.robot;

// SwerveModule.java
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {

  private final SparkFlex driveMotor;
  private final SparkFlex angleMotor;

  public SwerveModule(int driveID, int angleID) {
    driveMotor = new SparkFlex(driveID, MotorType.kBrushless);
    angleMotor = new SparkFlex(angleID, MotorType.kBrushless);

    SparkFlexConfig config = new SparkFlexConfig();
    driveMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    angleMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
