package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax; // Use SparkFlex if using Flex hardware
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SpindexerConstants;

public class Spindexer extends SubsystemBase {
    // If you are using a Spark Flex, change 'SparkMax' to 'SparkFlex' below
    private final SparkMax motor = new SparkMax(SpindexerConstants.kSpindexerMotorId, MotorType.kBrushless);
    private final SparkMaxConfig config = new SparkMaxConfig();

    public Spindexer() {
        /* * The new API uses a config object to set everything at once. 
         * This is much more reliable than setting individual parameters.
         */
        config.inverted(false);
        config.smartCurrentLimit(40); // Standard safety limit

        // Apply the configuration to the motor
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void spin() {
        motor.set(SpindexerConstants.kSpindexerSpeed);
    }

    public void stop() {
        motor.set(0);
    }
}