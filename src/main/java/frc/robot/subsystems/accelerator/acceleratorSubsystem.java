package frc.robot.subsystems.accelerator;

import com.revrobotics.spark.SparkFlex; // Assuming Flex, change to SparkMax if needed
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AcceleratorConstants;

public class acceleratorSubsystem extends SubsystemBase {
    private final SparkFlex motor = new SparkFlex(AcceleratorConstants.kAcceleratorMotorId, MotorType.kBrushless);

    private boolean spining = false;

    public acceleratorSubsystem() {
        SparkFlexConfig config = new SparkFlexConfig();
        config.smartCurrentLimit(40);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void spin() {
        motor.set(AcceleratorConstants.kAcceleratorSpeed);
        spining=true;
    }

    public void spin(double speed) {
        motor.set(speed);
        spining=true;
    }

    public void reverseSpin(){
        motor.set(-AcceleratorConstants.kAcceleratorSpeed);
        spining=true;
    }

    public void reverseSpin(double speed){
        motor.set(-speed);
        spining=true;
    }

    public void stop() {
        motor.set(0);
        spining=false;
    }

    public boolean getSpining(){
        return spining;
    }
}