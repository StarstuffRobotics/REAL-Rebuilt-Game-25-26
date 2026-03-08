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

    public LinearServo(int channel, double length, double speed) {
        super(channel);
        super.setBoundsMicroseconds(2000, 0, 1500, 0, 1000);
        this.m_length = length;
        this.m_speed = speed;
    }


    public void setPosition(double setpoint) {
        setPos = Math.max(0, Math.min(m_length, setpoint));
        double servoValue = setPos / m_length;
        set(servoValue);
    }

    public void updateCurPos() {
        double dt = Timer.getFPGATimestamp() - lastTime;
        lastTime = Timer.getFPGATimestamp();
        if (curPos > setPos + m_speed * dt) {
            curPos -= m_speed * dt;
        } else if (curPos < setPos - m_speed * dt) {
            curPos += m_speed * dt;
        } else {
            curPos = setPos;
        }
    }

    public double getAngle() {
        return ((curPos / m_length) * (MAX_ANGLE - MIN_ANGLE) + MIN_ANGLE);
    }

    public boolean isFinished() {
        return Math.abs(curPos - setPos) < 0.5;
    }
}