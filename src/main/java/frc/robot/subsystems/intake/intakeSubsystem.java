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

    //intake up down methods

    public void intakeUp(double rotations){
        pid.setReference(rotations, ControlType.kPosition); // Updated to use the motor's closed-loop controller
    }

    public void intakeDown(double rotations){
        pid.setReference(rotations, ControlType.kPosition); // Updated to use the motor's closed-loop controller
    }

    public void intakeStop(){
        updown_motor.stopMotor();
    }



    //roller methods


    public void rollerIn(double speed){
        roller_motor.set(speed); //need to test
    }

    public void rollerOut(double speed){
        roller_motor.set(-speed); //need to test
    }

    public void rollerStop(){
        roller_motor.stopMotor();
    }


}
  