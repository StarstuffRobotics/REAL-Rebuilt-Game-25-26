package frc.robot;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.CANcoderConfiguration;

public class SwerveModule {
    private final SparkFlex driveMotor;
    private final SparkFlex angleMotor;
    private final RelativeEncoder angleEncoder;
    private final SparkClosedLoopController anglePID;
    private final CANcoder absoluteEncoder;

    public SwerveModule(int driveID, int angleID, int canCoderID, double angleOffset, boolean steeringInverted) {
        driveMotor = new SparkFlex(driveID, MotorType.kBrushless);
        angleMotor = new SparkFlex(angleID, MotorType.kBrushless);
        absoluteEncoder = new CANcoder(canCoderID);

        CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
        canCoderConfig.MagnetSensor.MagnetOffset = angleOffset;
        absoluteEncoder.getConfigurator().apply(canCoderConfig);

        angleEncoder = angleMotor.getEncoder();
        anglePID = angleMotor.getClosedLoopController();

        SparkFlexConfig angleConfig = new SparkFlexConfig();
        angleConfig.closedLoop
            .pid(Constants.Swerve.angleP, Constants.Swerve.angleI, Constants.Swerve.angleD)
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(0, 2 * Math.PI);
        
        angleConfig.encoder.positionConversionFactor(Constants.Swerve.STEER_ROTATIONS_TO_RADIANS);
        angleConfig.inverted(steeringInverted); 
        angleConfig.smartCurrentLimit(40);

        SparkFlexConfig driveConfig = new SparkFlexConfig();
        driveConfig.encoder.positionConversionFactor(Constants.Swerve.DRIVE_ROTATIONS_TO_METERS);
        driveConfig.encoder.velocityConversionFactor(Constants.Swerve.DRIVE_ROTATIONS_TO_METERS / 60.0);
        driveConfig.inverted(false);
        driveConfig.smartCurrentLimit(60);

        angleMotor.configure(angleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Give the CANcoder a moment to broadcast before syncing
        try { Thread.sleep(50); } catch (Exception e) {}
        resetToAbsolute();
    }
   
   //Not sure this this is the right fix, need to test it.

   /* 
   public SwerveModule(int drivingCanId, int turningCanId, double chassisAngularOffset) {
        driveMotor = new SparkFlex(drivingCanId, MotorType.kBrushless);
        angleMotor = new SparkFlex(turningCanId, MotorType.kBrushless);
        absoluteEncoder = new CANcoder(drivingCanId); // Initialize absoluteEncoder

        CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
        canCoderConfig.MagnetSensor.MagnetOffset = chassisAngularOffset;
        absoluteEncoder.getConfigurator().apply(canCoderConfig);

        angleEncoder = angleMotor.getEncoder(); // Initialize angleEncoder
        anglePID = angleMotor.getClosedLoopController(); // Initialize anglePID
    }
        */


    public void resetToAbsolute() {
        double absolutePosition = absoluteEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI;
        angleEncoder.setPosition(absolutePosition);
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromRadians(angleEncoder.getPosition());
    }

    public void setState(SwerveModuleState state) {
        SwerveModuleState optimized = SwerveModuleState.optimize(state, getAngle());
        
        driveMotor.set(optimized.speedMetersPerSecond / Constants.Swerve.MAX_SPEED);

        // Convert -PI to PI range into 0 to 2PI range to match your config
        double angleToSet = optimized.angle.getRadians();
        if (angleToSet < 0) {
            angleToSet += 2 * Math.PI;
        }

        anglePID.setReference(angleToSet, ControlType.kPosition);
    }
    
    public SwerveModuleState getState() {
        double velocity = driveMotor.getEncoder().getVelocity();
        Rotation2d angle = getAngle();
        return new SwerveModuleState(velocity, angle);
    }

    public SwerveModulePosition getPosition() {

        // Implement the logic to return the SwerveModulePosition

        // Example:

        return new SwerveModulePosition(getDriveDistance(), getAngle());

    }

    public double getDriveDistance() {
        // Implement the logic to return the drive distance in meters
        return driveMotor.getEncoder().getPosition();
    }

    
    public double getRelativePosition() { return angleEncoder.getPosition(); }
    public double getAbsolutePosition() { return absoluteEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI; }
}