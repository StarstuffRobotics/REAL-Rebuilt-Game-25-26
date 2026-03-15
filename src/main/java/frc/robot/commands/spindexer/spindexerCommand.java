package frc.robot.commands.spindexer;

import frc.robot.Constants.SpindexerConstants;
import frc.robot.subsystems.spindexer.spindexerSubsystem;


public class spindexerCommand {
    private final spindexerSubsystem spindexer;

    public spindexerCommand(spindexerSubsystem spindexer){
        this.spindexer = spindexer;
    }

    public void spin(){
        spindexer.spin(SpindexerConstants.kSpindexerSpeed);
    }

    public void spin(double speed){
        spindexer.spin(speed);
    }

    public void reversedSpin(){
        spindexer.reversedSpin();
    }

    
    public void reversedSpin(double speed){
        spindexer.reversedSpin(speed);
    }

    public void stop(){
        spindexer.stop();
    }
}