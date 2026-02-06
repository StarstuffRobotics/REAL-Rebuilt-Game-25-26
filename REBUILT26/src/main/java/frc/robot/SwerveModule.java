package frc.robot;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {
    private final CANSparkFlex driveMotor;
    private final CANSparkFlex angleMotor;
    private final RelativeEncoder angleEncoder;
    private final SparkPIDController anglePID;

    public SwerveModule(int driveID, int angleID) {
        driveMotor = new CANSparkFlex(driveID, MotorType.kBrushless);
        angleMotor = new CANSparkFlex(angleID, MotorType.kBrushless);

        driveMotor.restoreFactoryDefaults();
        angleMotor.restoreFactoryDefaults();

        angleEncoder = angleMotor.getEncoder();
        anglePID = angleMotor.getPIDController();

        anglePID.setP(Constants.Swerve.angleP);
        anglePID.setI(Constants.Swerve.angleI);
        anglePID.setD(Constants.Swerve.angleD);

        // Conversion: Converts motor rotations to radians
        // 12.8 is a common swerve steering ratio (adjust if using MK4i/SDS)
        angleEncoder.setPositionConversionFactor(2 * Math.PI / 12.8); 

        angleMotor.burnFlash();
        driveMotor.burnFlash();
    }

    public Rotation2d getAngle() {
        return new Rotation2d(angleEncoder.getPosition());
    }

    public void setState(SwerveModuleState state) {
        // Prevents wheels from snapping back to 0 when stopped
        if (Math.abs(state.speedMetersPerSecond) < 0.01) {
            driveMotor.set(0);
            return;
        }

        SwerveModuleState optimized = SwerveModuleState.optimize(state, getAngle());
        driveMotor.set(optimized.speedMetersPerSecond / Constants.Swerve.MAX_SPEED);
        anglePID.setReference(optimized.angle.getRadians(), CANSparkFlex.ControlType.kPosition);
    }
}