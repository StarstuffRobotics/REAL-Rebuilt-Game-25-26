
package frc.robot.subsystems.spindexer;

import com.revrobotics.spark.SparkMax; // Use SparkFlex if using Flex hardware
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class spindexerSubsystem extends SubsystemBase {
    // If you are using a Spark Flex, change 'SparkMax' to 'SparkFlex' below
    private final SparkMax motor = new SparkMax(SpindexerConstants.kSpindexerMotorId, MotorType.kBrushless);
    private final SparkMaxConfig config = new SparkMaxConfig();

    private boolean spindexerSpinning = false;

    public spindexerSubsystem() {
        /* * The new API uses a config object to set everything at once. 
         * This is much more reliable than setting individual parameters.
         */
        config.inverted(false);
        config.smartCurrentLimit(40); // Standard safety limit

        // Apply the configuration to the motor
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    
    public void periodic(){
        SmartDashboard.putBoolean("Spindexer", spindexerSpinning);

    }

    public void spin() {
        if (!spindexerSpinning) {
            motor.set(SpindexerConstants.kSpindexerSpeed);
            spindexerSpinning = true;
        }else{
            stop();
            spindexerSpinning = false;
        }
        
    }

    public void spin(double speed) {
        if (!spindexerSpinning) {
            motor.set(speed);
            spindexerSpinning = true;
        }else{
            stop();
            spindexerSpinning = false;
        }
    }

    public void reversedSpin() {
        if (!spindexerSpinning) {
            motor.set(-SpindexerConstants.kSpindexerSpeed);
        }else{
            stop();
        }
    }

    public void reversedSpin(double speed) {
        if (!spindexerSpinning) {
            motor.set(-speed);
        }else{
            stop();
        }
    }

    public void stop() {
        motor.set(0);
        spindexerSpinning = false;
    }

    public boolean getSpindexerSpinning() {
        return spindexerSpinning;
    }
}
