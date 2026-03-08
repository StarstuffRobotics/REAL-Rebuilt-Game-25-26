package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;

public class LinearServo extends Servo {
    double m_speed;
    double m_length;
    double setPos;
    double curPos;
    double lastTime = 0;
    double MAX_ANGLE = 40;
    double MIN_ANGLE = 15;
    private static LinearServo hood1 = new LinearServo(0, 100, 50); // Example parameters: channel 1, length 100mm, speed 50mm/s
    private static LinearServo hood2 = new LinearServo(1, 100, 50);


    


    /**
     * Parameters for L16-R Actuonix Linear Actuators
     *
     * @param channel PWM channel used to control the servo
     * @param length max length of the servo [mm]
     * @param speed max speed of the servo [mm/second]
     */
    public LinearServo(int channel, double length, double speed) {
        super(channel); // Call the superclass constructor with the channel parameter
        
        super.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);;  
        
        this.m_length = length;
        this.m_speed = speed;
    }

    /**
     * Set the target position of the servo
     *
     * @param setpoint the target position [mm]
     */

    public static LinearServo getHood1(){
        return hood1;
    }

    public static LinearServo getHood2(){
        return hood2;
    }

    public void setPosition(double setpoint) {
        // setPos = MathUtil.clamp(setpoint, 0, m_length);
        // setSpeed((setPos / m_length * 2) - 1);

        setPos= Math.max(0,Math.min(m_length, setpoint)); 
        double servoValue = setPos / m_length; 

        set(servoValue); 
    }

    /**
     * Run this in periodic() to update position estimation
     */
    public void updateCurPos() {
        double dt = Timer.getFPGATimestamp() - lastTime;
        lastTime = Timer.getFPGATimestamp(); // ✅ Fixed - was never updating lastTime!
        if (curPos > setPos + m_speed * dt) {
            curPos -= m_speed * dt;
        } else if (curPos < setPos - m_speed * dt) {
            curPos += m_speed * dt;
        } else {
            curPos = setPos;
        }
    }

    /**
     * Get current position of the servo
     * Must be calling updateCurPos() periodically
     *
     * @return Servo Position [mm]
     */
    public double getAngle() {
        return ((curPos/m_length)*(MAX_ANGLE-MIN_ANGLE)+MIN_ANGLE); // ✅ Fixed - was missing return statement!
    }

    /**
     * Checks if servo is at its target position
     * Must be calling updateCurPos() periodically
     *
     * @return true when servo is at its target
     */
    public boolean isFinished() {
        return Math.abs(curPos - setPos) < 0.5; // ✅ Fixed - use tolerance instead of ==
    }
}