package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;

public class FixedHubTurretAim {


   
    // HUB POSITION (meters) – change to real field coordinates
    private double HUB_X;
    private double HUB_Y;

    
   

    // Turret interface so this class works with your subsystem
    public interface TurretInterface {

        // turret encoder angle in radians
        double getTurretAngleRad();

        // command turret to a new angled
    
        void setTurretAngleRad(double angle);
    }

    

    private TurretInterface turret;


    public FixedHubTurretAim(TurretInterface turret, boolean team) {//true for blue, false for red
        this.turret = turret;
        if (team) {
            HUB_X = 4.612707; // Blue hub X coordinate
            HUB_Y = 3.959698; // Blue hub Y coordinate
        } else {
            HUB_X = 11.865873; // Red hub X coordinate
            HUB_Y = 3.985418; // Red hub Y coordinate
        }
    }

    /**
     * Calculates how far the turret must rotate to face the hub.
     */
    public double getTurretRotationDelta() {

        // If Limelight doesn't see any tag we cannot get pose
        if (!LimelightHelpers.getTV("limelight")) {
            return 0;
        }

        // Get robot pose from Limelight MegaTag
        Pose2d robotPose =
            LimelightHelpers.getBotPose2d_wpiBlue("limelight");

        double robotX = robotPose.getX();
        double robotY = robotPose.getY();

        // 1️⃣ Angle from robot → hub (field coordinates)
        double angleToHub =
            Math.atan2(HUB_Y - robotY, HUB_X - robotX);

        // 2️⃣ Robot heading
        double robotHeading =
            robotPose.getRotation().getRadians();

        // 3️⃣ Convert to robot-relative angle
        double robotRelativeAngle =
            normalize(angleToHub - robotHeading);

        // 4️⃣ Current turret angle
        double turretAngle =
            turret.getTurretAngleRad();

        // 5️⃣ Required turret rotation
        double turretDelta =
            normalize(robotRelativeAngle - turretAngle);

        return turretDelta;
    }

    /**
     * Automatically rotates turret toward hub.
     */
    public void aimTurret() {

        double delta = getTurretRotationDelta();

        double currentAngle = turret.getTurretAngleRad();
        if(currentAngle + delta>Math.toRadians(-60)&&currentAngle + delta<Math.toRadians(60)){
            turret.setTurretAngleRad(currentAngle + delta);
        }else{
            turret.setTurretAngleRad((currentAngle + delta)-60);
        }
    }

    private double normalize(double angle) {

        while (angle > Math.PI)
            angle -= 2 * Math.PI;

        while (angle < -Math.PI)
            angle += 2 * Math.PI;

        return angle;
    }
}