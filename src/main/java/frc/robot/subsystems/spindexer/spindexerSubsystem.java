
package frc.robot.subsystems.spindexer;

import com.revrobotics.spark.SparkMax; // Use SparkFlex if using Flex hardware
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class spindexerSubsystem extends SubsystemBase {
    // If you are using a Spark Flex, change 'SparkMax' to 'SparkFlex' below
    private final SparkMax motor = new SparkMax(spindexerConstants.kSpindexerMotorId, MotorType.kBrushless);
    private final SparkMaxConfig config = new SparkMaxConfig();

    public spindexerSubsystem() {
        /* * The new API uses a config object to set everything at once. 
         * This is much more reliable than setting individual parameters.
         */
        config.inverted(false);
        config.smartCurrentLimit(40); // Standard safety limit

        // Apply the configuration to the motor
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void spin() {
        motor.set(spindexerConstants.kSpindexerSpeed);
    }

    public void spin(double speed) {
        motor.set(speed);
    }

    public void reversedSpin() {
        motor.set(-spindexerConstants.kSpindexerSpeed);
    }

    public void reversedSpin(double speed) {
        motor.set(-speed);
    }

    public void stop() {
        motor.set(0);
    }
}
