package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.intakeSubsystem;

public class intakeCommands extends Command {
    //the commands that control the methods from intakeSubsystem. bound to some of the buttons in configureBindings.
    private final intakeSubsystem intake;

    public intakeCommands(intakeSubsystem intake){
        this.intake = intake;
    }


    //intake contole 
    public void intakeUp(double pos){
        intake.intakeUp(pos);
    }

    public void intakeDown(double pos){
        intake.intakeDown(pos);
    }

    public void intakeStop(){
        intake.intakeStop();
    }


    //intake roller control
    public void rollerIn(double speed){

        if (!intake.getIsUp()){
            intake.rollerIn(speed);
        }
    }

    public void rollerOut(double speed){
        if (!intake.getIsUp()){
            intake.rollerOut(-speed);
        }
    }

    public void rollerStop(){
        intake.rollerStop();
    }
}
