package frc.robot;

import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.Constants.Swerve;

public final class Configs {
    public static final class MAXSwerveModule {
        public static final SparkFlexConfig drivingConfig = new SparkFlexConfig();
        public static final SparkFlexConfig turningConfig = new SparkFlexConfig();

        static {
            // Drive Motor
            drivingConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(60);
            drivingConfig.encoder
                .positionConversionFactor(Swerve.DRIVE_ROTATIONS_TO_METERS)
                .velocityConversionFactor(Swerve.DRIVE_ROTATIONS_TO_METERS / 60.0);
            drivingConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(0.04, 0, 0)
                .velocityFF(0.12);

            // Turn Motor
            turningConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40);
            turningConfig.encoder
                .positionConversionFactor(Swerve.STEER_ROTATIONS_TO_RADIANS)
                .velocityConversionFactor(Swerve.STEER_ROTATIONS_TO_RADIANS / 60.0);
            turningConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(Swerve.angleP, Swerve.angleI, Swerve.angleD)
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(0, 2 * Math.PI);
        }
    }
}