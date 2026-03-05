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

    private boolean spinning = false;

    public acceleratorSubsystem() {
        SparkFlexConfig config = new SparkFlexConfig();
        config.smartCurrentLimit(40);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void spinToggle(){
        if (!spinning) {
            spin();
            spinning=true;
        }else{
            stop();
        }
    }

    public void spin() {
        if (!spinning) {
            motor.set(-1*AcceleratorConstants.kAcceleratorSpeed);
            spinning=true;
        }else{
            stop();
        }
    }

    public void spin(double speed) {
        if (!spinning) {
            motor.set(-speed);
            spinning=true;
        }else{
            stop();
        }
    }

    public void reverseSpin(){
        if (!spinning) {
            motor.set(AcceleratorConstants.kAcceleratorSpeed);
            spinning=true;
        }else{
            stop();
        }
    }

    public void reverseSpin(double speed){
        if (!spinning) {
            motor.set(-speed);
            spinning=true;
        }else{
            stop();
        }
    }

    public void stop() {
        motor.set(0);
        spinning=false;
    }

    public boolean getSpining(){
        return spinning;
    }
}