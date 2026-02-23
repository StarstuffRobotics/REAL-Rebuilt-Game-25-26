package frc.robot.subsystems;



//FIXED HOOD, TURRET SPINS, NEED TO CONTROL TRAJECTORY USING SPEED AND SPIN
public class TurretSubsystem {
    public boolean simAbleToIntake() {
        // Return true if the turret is in a position to intake
        return true; // Example implementation
    }
    
    public void simIntake() {
        // Simulate the turret intaking fuel
        System.out.println("Simulating turret intake...");
    } 
}