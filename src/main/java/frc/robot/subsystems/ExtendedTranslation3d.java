
package frc.robot.subsystems;
//
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

public class ExtendedTranslation3d extends Translation3d {

    public ExtendedTranslation3d(double x, double y, double z) {
        super(x, y, z);
    }
    public ExtendedTranslation3d(Translation3d translation) {
        super(translation.getX(), translation.getY(), translation.getZ()); // Assuming Z is 0 for 2D translations
    }
    public ExtendedTranslation3d(Translation2d translation) {
        super(translation.getX(), translation.getY(),0);
    }

    public ExtendedTranslation3d(ExtendedTranslation2d translation) {
        super(translation.getX(), translation.getY(), 0);
    }
    public double dot(Translation3d other) {
        return this.getX() * other.getX() + this.getY() * other.getY() + this.getZ() * other.getZ();
    }
    public double dot(ExtendedTranslation3d other) {
        return this.getX() * other.getX() + this.getY() * other.getY();
    }

    public double dot(Translation2d other) {
        return this.getX() * other.getX() + this.getY() * other.getY();
    }
    
    public double dot(ExtendedTranslation2d other) {
        return this.getX() * other.getX() + this.getY() * other.getY();
    }
    @Override
    public ExtendedTranslation3d plus(Translation3d other) {
        return new ExtendedTranslation3d(this.getX() + other.getX(), this.getY() + other.getY(), this.getZ() + other.getZ());
    }

    @Override
    public ExtendedTranslation3d minus(Translation3d other) {
        return new ExtendedTranslation3d(this.getX() - other.getX(), this.getY() - other.getY(), this.getZ() - other.getZ());
    }

    @Override
    public ExtendedTranslation3d times(double scalar) {
        return new ExtendedTranslation3d(this.getX() * scalar, this.getY() * scalar, this.getZ() * scalar);
    }

    @Override
    public ExtendedTranslation3d div(double scalar) {
        return new ExtendedTranslation3d(this.getX() / scalar, this.getY() / scalar, this.getZ() / scalar);
    }
}
