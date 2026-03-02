package frc.robot.commands.spindexer;

import frc.robot.subsystems.spindexer.spindexerSubsystem;


public class spindexerCommands {
    private final spindexerSubsystem spindexer;

    public spindexerCommands(spindexerSubsystem spindexer){
        this.spindexer = spindexer;
    }

    public void spin(){
        spindexer.spin();
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
