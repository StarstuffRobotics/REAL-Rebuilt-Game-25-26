package frc.robot;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveConstants;

public class SwerveModule {
    private final SparkFlex driveMotor;
    private final SparkFlex steerMotor;
    private final CANcoder absoluteEncoder;
    private final double encoderOffset;

    public SwerveModule(int driveId, int steerId, int encoderId, double offset) {
        this.encoderOffset = offset;
        driveMotor = new SparkFlex(driveId, MotorType.kBrushless);
        steerMotor = new SparkFlex(steerId, MotorType.kBrushless);
        absoluteEncoder = new CANcoder(encoderId);

        SparkFlexConfig driveConfig = new SparkFlexConfig();
        SparkFlexConfig steerConfig = new SparkFlexConfig();

        driveConfig.smartCurrentLimit(40).idleMode(SparkFlexConfig.IdleMode.kBrake);
        driveConfig.encoder.positionConversionFactor(DriveConstants.kDrivePositionConversion)
                           .velocityConversionFactor(DriveConstants.kDriveVelocityConversion);
        steerConfig.smartCurrentLimit(20).idleMode(SparkFlexConfig.IdleMode.kBrake)
                   .inverted(true); // MK4i Specific
        steerConfig.encoder.positionConversionFactor(DriveConstants.kSteerPositionConversion);
        steerConfig.closedLoop.feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder).pid(0.5, 0, 0);

        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        steerMotor.configure(steerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(driveMotor.getEncoder().getVelocity(), getRotation());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveMotor.getEncoder().getPosition(), getRotation());
    }

    public Rotation2d getRotation() {
        double pos = absoluteEncoder.getAbsolutePosition().getValueAsDouble() - encoderOffset;
        return Rotation2d.fromRotations(pos);
    }

    public void setDesiredState(SwerveModuleState state) {
        state = SwerveModuleState.optimize(state, getRotation());
        driveMotor.set(state.speedMetersPerSecond / 4.5); // 4.5 is max speed m/s estimate
        steerMotor.getClosedLoopController().setReference(state.angle.getRadians(), SparkFlex.ControlType.kPosition);
    }
}