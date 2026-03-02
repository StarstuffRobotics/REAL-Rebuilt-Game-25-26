// package frc.robot;

// import com.revrobotics.spark.SparkFlex;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.SparkBase;
// import com.revrobotics.spark.config.SparkFlexConfig;
// import com.revrobotics.spark.SparkBase.PersistMode;
// import com.revrobotics.spark.SparkBase.ResetMode;
// import com.ctre.phoenix6.hardware.CANcoder;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.kinematics.SwerveModulePosition;
// import edu.wpi.first.math.kinematics.SwerveModuleState;
// import frc.robot.Constants.DriveConstants;

// public class SwerveModule {
//     private final SparkFlex driveMotor;
//     private final SparkFlex steerMotor;
//     private final CANcoder absoluteEncoder;
//     private final double encoderOffset;

//     public SwerveModule(int driveId, int steerId, int encoderId, double offset) {
//         this.encoderOffset = offset;
//         driveMotor = new SparkFlex(driveId, MotorType.kBrushless);
//         steerMotor = new SparkFlex(steerId, MotorType.kBrushless);
//         absoluteEncoder = new CANcoder(encoderId);

//         SparkFlexConfig driveConfig = new SparkFlexConfig();
//         SparkFlexConfig steerConfig = new SparkFlexConfig();

//         // Drive Config
//         driveConfig.smartCurrentLimit(40).idleMode(SparkFlexConfig.IdleMode.kBrake);
//         driveConfig.encoder.positionConversionFactor(DriveConstants.kDrivePositionConversion);
//         driveConfig.encoder.velocityConversionFactor(DriveConstants.kDriveVelocityConversion);
        
//         // Steer Config (CRITICAL: Added PID and Closed Loop setup)
//         steerConfig.smartCurrentLimit(20).idleMode(SparkFlexConfig.IdleMode.kBrake).inverted(true);
//         steerConfig.closedLoop.pid(0.5, 0.0, 0.0); // You will need to tune these P values!
//         steerConfig.closedLoop.positionConversionFactor(2 * Math.PI); // Radians per rotation

//         driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
//         steerMotor.configure(steerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
//     }

//     public Rotation2d getRotation() {
//         // Ensure this returns a value in rotations (0-1)
//         double pos = absoluteEncoder.getAbsolutePosition().getValueAsDouble() - encoderOffset;
//         return Rotation2d.fromRotations(pos);
//     }

//     public SwerveModuleState getState() {
//         return new SwerveModuleState(driveMotor.getEncoder().getVelocity(), getRotation());
//     }

//     public SwerveModulePosition getPosition() {
//         return new SwerveModulePosition(driveMotor.getEncoder().getPosition(), getRotation());
//     }

//     public void setDesiredState(SwerveModuleState state) {
//         // Optimize state to ensure module doesn't spin more than 90 degrees
//         state = SwerveModuleState.optimize(state, getRotation());
        
//         // Feedforward the drive motor (remove the /4.5 and use proper voltage/velocity control later)
//         driveMotor.set(state.speedMetersPerSecond / 4.5); 
        
//         // Use closed loop position control for steering
//         steerMotor.getClosedLoopController().setReference(state.angle.getRadians(), SparkBase.ControlType.kPosition);
//     }
// }