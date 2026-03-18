package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HoodSubsystem extends SubsystemBase {
    
    private final Servo m_servoLeft;
    private final Servo m_servoRight;

    public HoodSubsystem() {
        m_servoLeft  = new Servo(0);
        m_servoRight = new Servo(1);

        // Linear servos typically use a simple min/max pulse range.
        // Adjust these bounds to match your specific servo's datasheet.
        m_servoLeft.setBoundsMicroseconds(2000, 1500, 1500, 1500, 1000);
        m_servoRight.setBoundsMicroseconds(2000, 1500, 1500, 1500, 1000);
    }

    /** Stops both servos (holds position). */
    public void stopHood() {
        m_servoLeft.setSpeed(0);
        m_servoRight.setSpeed(0);
    }

    /** Returns the current angle of the left servo as a proxy for hood angle. */
    public double getHoodAngle() {
        return m_servoLeft.getAngle();
    }

    /** Drives both servos to their 0° position. */
    public void setHoodZero() {
        m_servoLeft.setAngle(0);
        m_servoRight.setAngle(0);
    }

    /** Drives both servos to their 180° position. */
    public void setHoodMax() {
        m_servoLeft.setAngle(180);
        m_servoRight.setAngle(180);
    }

    /**
     * Sets both servos to the same target angle.
     * The right servo is mirrored (180 - angle) to account for
     * it being physically flipped on the opposite side of the hood.
     *
     * @param angle target angle in degrees [0, 180]
     */
    public void setHoodAngle(double angle) {
        m_servoLeft.setAngle(angle);
        m_servoRight.setAngle(180 - angle); // mirror for opposite-side mounting
    }
}