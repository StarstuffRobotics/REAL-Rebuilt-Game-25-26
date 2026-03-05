package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class intakeSubsystem {
        //set up for the intake, motors and methods to controle the motors.
    public SparkFlex updown_motor = new SparkFlex(20, MotorType.kBrushless);
    public SparkFlex roller_motor = new SparkFlex(21, MotorType.kBrushless);
    SparkClosedLoopController pid = updown_motor.getClosedLoopController();


    private boolean isup = true;
    private boolean rollerOn = false;
    private boolean rollerDirection = true; // true for in, false for out

    public intakeSubsystem() {
        SparkFlexConfig config = new SparkFlexConfig();
        config.smartCurrentLimit(40);
        updown_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        roller_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
   
    }


    //intake up down methods
    
    public void toggleDirection(){
        isup=!isup;
    }

    public void intakeUp(){
        
            //pid.setSetpoint(rotations, ControlType.kPosition); // Updated to use the motor's closed-loop controller
            updown_motor.set(-5);
            toggleDirection();
        
    }

    public void intakeDown(){
        //pid.setSetpoint(rotations, ControlType.kPosition); // Updated to use the motor's closed-loop controller
        updown_motor.set(5);
        toggleDirection();
    }

    public boolean getIsUp(){
        return isup;
    }

    public void intakeUpDown(){
        if (isup){
            intakeDown(); // Move to the down position
        } else {
            intakeUp(); // Move to the up position
        }
    }

    public void intakeStop(){
        updown_motor.stopMotor();
    }

    //roller methods


    public void rollerInOff(){
        if (!rollerOn || !rollerDirection){ //if the roller is off or currently out, run the roller in
            rollerIn(); // Adjust the speed as needed
           
        }else{
            rollerStop();
        }
    }

    public void rollerIn(){
        if (!isup){ //only run the roller if the intake is down
            roller_motor.set(1); //need to test
            rollerOn= true;
            rollerDirection = true;
        }
    }

    public void rollerOut(){
        if (!isup){
            roller_motor.set(-1); //need to test
            rollerOn = true;
            rollerDirection = false;
        }
    }

    public boolean getRollerOn(){
        return rollerOn;
    }

    public boolean getRollerDirection(){
        return rollerDirection;
    }

    public void rollerStop(){
        roller_motor.stopMotor();
        rollerOn = false;
    }


}
  