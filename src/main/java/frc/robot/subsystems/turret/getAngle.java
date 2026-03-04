package frc.robot.subsystems.turret;

public class getAngle {


    ///
    /// 
    /// Note for later : hoodAngle ≈ 14 + 1.6 × distanceFeet
    /// that is an aproximation for the deiered hood angle based on distance
    /// here is an aprocimate table:
    /// 2–4 ft → 15°–22°
    // 5–8 ft → 25°–31°
    // 9–12 ft → 32°–35°
    // 13–15 ft → 36°–37°
    /// 
    
    
    private static final double GRAVITY = 32.17;

    private static final double K_CALIBRATION = 0.85;

    private static final double WHEEL_RADIUS_FT = 0.166;

    private static final double MIN_ANGLE_DEG = 15.0;
    private static final double MAX_ANGLE_DEG = 40.0;

    private static final double LAUNCH_HEIGHT_FT = 1.775;
    private static final double TARGET_HEIGHT_FT = 6.0;
    private static final double DELTA_H = TARGET_HEIGHT_FT - LAUNCH_HEIGHT_FT;

    
    private static final double MAX_EXIT_VELOCITY_FT_PER_SEC = 31.54;

    public static double calculateTargetRPM(double distanceFeet, double hoodAngleDegrees) {


        if (distanceFeet <= 0) return -1;

        double clampedAngle = Math.max(MIN_ANGLE_DEG,
                            Math.min(MAX_ANGLE_DEG, hoodAngleDegrees));

        double theta = Math.toRadians(clampedAngle);
        
        double numerator = GRAVITY * Math.pow(distanceFeet, 2);

        double denominator =
                2 * Math.pow(Math.cos(theta), 2)
                * (distanceFeet * Math.tan(theta) - DELTA_H);

        
        if (denominator <= 0.01) return -1;
        

        double v0 = Math.sqrt(numerator / denominator);
        
        if (v0 > MAX_EXIT_VELOCITY_FT_PER_SEC) {
            v0 = MAX_EXIT_VELOCITY_FT_PER_SEC;
        }
        
        double rpm =
                ((v0 / K_CALIBRATION) / WHEEL_RADIUS_FT)
                * 60 / (2 * Math.PI);

        return rpm;
    }
}
