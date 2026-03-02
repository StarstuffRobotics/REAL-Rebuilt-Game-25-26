package frc.robot.commands.Intake;

import frc.robot.subsystems.intake.intakeSubsystem;

public class intakeCommands {
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


    //intake roller control
    public void rollerIn(double speed){
        intake.rollerIn(speed);
    }

    public void rollerOut(double speed){
        intake.rollerOut(-speed);
    }

}
