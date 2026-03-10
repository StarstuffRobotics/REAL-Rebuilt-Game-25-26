package frc.robot.subsystems.turret;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.FixedHubTurretAim;

public class rotationSubsystem extends SubsystemBase
        implements FixedHubTurretAim.TurretInterface {

    private FixedHubTurretAim turretAimer;

    private final String LIMELIGHT_NAME = "limelight-turret"; 

    private SparkFlex turret_motor1; 
    
    // Soft limit constants (tune to your robot's physical range)
    private static final double TURRET_MAX_POSITION = 60.0;  // degrees
    private static final double TURRET_MIN_POSITION = -60.0; // degrees
    private double Tx;
    private double Ty;
    private boolean Tv;
    private boolean track = false; // Set to true to enable tracking in periodic()
    

    private final PIDController turnController = new PIDController(
        Constants.LimelightConstants.kTurnP, 
        Constants.LimelightConstants.kTurnI, 
        Constants.LimelightConstants.kTurnD
    );
    
   
    public rotationSubsystem() {
        turnController.setTolerance(1.5); // degrees
        turret_motor1 = new SparkFlex(24, MotorType.kBrushless);
        turretAimer = new FixedHubTurretAim(this, true); // true for blue team, false for red team

    }

    @Override
    public double getTurretAngleRad() {
    
        return turret_motor1.getEncoder().getPosition() * Constants.rotationConstants.ENCODER_DEGREES_PER_ROTATION;
    }
    
    @Override
    public void setTurretAngleRad(double angle) {
    
        double current = getTurretAngleRad();
    
        double error = angle - current;
    
        double kP = 0.6;
    
        turret_motor1.set(kP * error);
    }

    @Override
    public void periodic() {    
        if (track){
            allignTurret();
        }
    }

    public void toogleTackingTurret(){
        track = !track;
    }

    public void allignTurret() {

        turretAimer.aimTurret();
    }

    public void stopRotation(){
        turret_motor1.stopMotor();
    }

    public void manualRotateRight(){
        turret_motor1.set(0.05);
    }

    public void manualRotateLeft(){
        turret_motor1.set(-0.05);
    }


}