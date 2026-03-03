package frc.robot.commands.accelerator;

import frc.robot.subsystems.accelerator.acceleratorSubsystem;

public class acceleratorCommands {
    private final acceleratorSubsystem acceleratorSubsystem;

    public acceleratorCommands(acceleratorSubsystem acceleratorSubsystem) {
        this.acceleratorSubsystem = acceleratorSubsystem;
    }

    public void spin() {
        acceleratorSubsystem.spin();
    }

    public void spin(double speed) {
        acceleratorSubsystem.spin();
    }

    public void reverseSpin() {
        acceleratorSubsystem.reverseSpin();
    }

    public void reverseSpin(double speed) {
        acceleratorSubsystem.reverseSpin(speed);
    }

    public void stop() {
        acceleratorSubsystem.stop();
    }
}
