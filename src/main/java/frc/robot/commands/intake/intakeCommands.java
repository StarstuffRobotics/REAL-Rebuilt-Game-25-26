package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.intakeSubsystem;

public class intakeCommands extends Command {
    //the commands that control the methods from intakeSubsystem. bound to some of the buttons in configureBindings.
    private final intakeSubsystem intake;

    public intakeCommands(intakeSubsystem intake){
        this.intake = intake;
    }


    //intake contole 
    public void intakeUpDown(){
        intake.intakeUpDown();
    }

    public void intakeStop(){
        intake.intakeStop();
    }
   
    

    //a;kdhsf 


    //intake roller control

    public void rollerInOff(){
        intake.rollerInOff();
    }

    public void rollerIn(){
        intake.rollerIn();
    }

    public void rollerOut(){  
        intake.rollerOut();
    }

    public void rollerStop(){
        intake.rollerStop();
    }
}