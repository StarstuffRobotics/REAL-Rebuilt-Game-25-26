package frc.robot;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {
  // TODO: Replace with your measured gear ratio and wheel diameter values.
  private static final double DRIVE_METERS_PER_ROTATION = 0.10;
  private static final double TURN_RADIANS_PER_ROTATION = 2.0 * Math.PI;
  private static final double DRIVE_KP = 0.18;
  private static final double TURN_KP = 1.8;

  private final SparkFlex driveMotor;
  private final SparkFlex turnMotor;
  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turnEncoder;
  private final SparkClosedLoopController driveController;
  private final SparkClosedLoopController turnController;

  private final int driveMotorId;
  private final int turnMotorId;
  private SwerveModuleState desiredState = new SwerveModuleState();

  public SwerveModule(int driveMotorId, int turnMotorId) {
    this.driveMotorId = driveMotorId;
    this.turnMotorId = turnMotorId;

    driveMotor = new SparkFlex(driveMotorId, MotorType.kBrushless);
    turnMotor = new SparkFlex(turnMotorId, MotorType.kBrushless);

    driveEncoder = driveMotor.getEncoder();
    turnEncoder = turnMotor.getEncoder();
    driveController = driveMotor.getClosedLoopController();
    turnController = turnMotor.getClosedLoopController();

    SparkFlexConfig driveConfig = new SparkFlexConfig();
    driveConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(60);
    driveConfig.encoder
        .positionConversionFactor(DRIVE_METERS_PER_ROTATION)
        .velocityConversionFactor(DRIVE_METERS_PER_ROTATION / 60.0);
    driveConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(DRIVE_KP, 0.0, 0.0);

    SparkFlexConfig turnConfig = new SparkFlexConfig();
    turnConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(30);
    turnConfig.encoder
        .positionConversionFactor(TURN_RADIANS_PER_ROTATION)
        .velocityConversionFactor(TURN_RADIANS_PER_ROTATION / 60.0);
    turnConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(TURN_KP, 0.0, 0.0)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(-Math.PI, Math.PI);

    driveMotor.configure(
        driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    turnMotor.configure(
        turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setState(SwerveModuleState state) {
    SwerveModuleState optimizedState = new SwerveModuleState(state.speedMetersPerSecond, state.angle);
    optimizedState.optimize(Rotation2d.fromRadians(turnEncoder.getPosition()));
    desiredState = optimizedState;

    driveController.setReference(optimizedState.speedMetersPerSecond, ControlType.kVelocity);
    turnController.setReference(optimizedState.angle.getRadians(), ControlType.kPosition);
  }

  public SwerveModuleState getDesiredState() {
    return desiredState;
  }

  public int getDriveMotorId() {
    return driveMotorId;
  }

  public int getTurnMotorId() {
    return turnMotorId;
  }
}
