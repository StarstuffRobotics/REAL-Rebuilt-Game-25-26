package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;


public class ExtendedTranslation2d extends Translation2d{
    public ExtendedTranslation2d(double x, double y) {
        super(x, y);
    }
    public ExtendedTranslation2d(Translation2d translation) {
        super(translation.getX(), translation.getY());
    }

    public ExtendedTranslation2d(ExtendedTranslation2d translation) {
        super(translation.getX(), translation.getY());
    }

    public ExtendedTranslation2d(Translation3d translation) {
        super(translation.getX(), translation.getY());
    }
    public ExtendedTranslation2d(ExtendedTranslation3d translation) {
        super(translation.getX(), translation.getY());
    }

    public double dot(Translation2d other) {
        return this.getX() * other.getX() + this.getY() * other.getY();
    }
    
    public double dot(ExtendedTranslation2d other) {
        return this.getX() * other.getX() + this.getY() * other.getY();
    }

    public double dot(ExtendedTranslation3d other) {
        return this.getX() * other.getX() + this.getY() * other.getY();
    }
    public double dot(Translation3d other) {
        return this.getX() * other.getX() + this.getY() * other.getY();
    }

}