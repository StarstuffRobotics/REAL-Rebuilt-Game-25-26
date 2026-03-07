package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;
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


    /**
     * Parameters for L16-R Actuonix Linear Actuators
     *
     * @param channel PWM channel used to control the servo
     * @param length max length of the servo [mm]
     * @param speed max speed of the servo [mm/second]
     */
    public LinearServo(int channel, int length, int speed) {
        super(channel);
        // Removed setBounds as it is not defined in the Servo class
        m_length = length;
        m_speed = speed;
    }

    /**
     * Set the target position of the servo
     *
     * @param setpoint the target position [mm]
     */

    public void setPosition(double setpoint) {
        setPos = MathUtil.clamp(setpoint, 0, m_length);
        setSpeed((setPos / m_length * 2) - 1);
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