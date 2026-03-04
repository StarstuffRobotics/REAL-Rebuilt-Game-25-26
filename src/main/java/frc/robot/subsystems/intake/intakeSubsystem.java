package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class intakeSubsystem {
        //set up for the intake, motors and methods to controle the motors.
    private SparkFlex updown_motor = new SparkFlex(20, MotorType.kBrushless);
    private SparkFlex roller_motor = new SparkFlex(21, MotorType.kBrushless);
    SparkClosedLoopController pid = updown_motor.getClosedLoopController();

    private boolean isup = false;
    private boolean rollerOn = false;
    private boolean rollerDirection = true; // true for in, false for out

    //intake up down methods
    
    public void intakeUp(double rotations){
        pid.setReference(rotations, ControlType.kPosition); // Updated to use the motor's closed-loop controller
        isup = true;
    }

    public void intakeDown(double rotations){
        pid.setReference(rotations, ControlType.kPosition); // Updated to use the motor's closed-loop controller
        isup = false;
    }

    public boolean getIsUp(){
        return isup;
    }

    public void intakeUpDown(){
        if (isup){
            intakeDown(0); // Move to the down position
        } else {
            intakeUp(1); // Move to the up position
        }
    }

    public void intakeStop(){
        updown_motor.stopMotor();
    }

    //roller methods


    public void rollerInOff(){
        if (!rollerOn || !rollerDirection){ //if the roller is off or currently out, run the roller in
            rollerIn(0.5); // Adjust the speed as needed
           
        }else{
            rollerStop();
        }
    }

    public void rollerIn(double speed){
        if (!isup){ //only run the roller if the intake is down
            roller_motor.set(speed); //need to test
            rollerOn= true;
            rollerDirection = true;
        }
    }

    public void rollerOut(double speed){
        if (!isup){
            roller_motor.set(-speed); //need to test
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
  