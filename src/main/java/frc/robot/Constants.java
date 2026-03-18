// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(27.5)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(14.5);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 20; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND        = 0.05;
    public static final double LEFT_Y_DEADBAND = 0.05;
    public static final double RIGHT_X_DEADBAND = 0.05;
    public static final double LEFT_X_DEADBAND = 0.05;
    public static final double TURN_CONSTANT    = 6;
  }
  

//All this is in inches and fake numbers for now
  public static class LimelightConstants
  {

    public static final double limelightMountAngleDegrees = 0;
    public static final double limelightTurretOffsetX = 0;
    public static final double limelightTurretOffsetY = 0;
    public static final double limelightTurretHeight = 18.5;
    public static final double kTurnP = 0.1;
    public static final double kTurnI = 0.0;
    public static final double kTurnD = 0.1;
  }


public static class hoodConstants{
  public final static int SERVO_PORT1 = 0;
  public final static int SERVO_PORT2 = 1;
}

public static class FieldConstants {
  public static final int TARGET_HEIGHT_FEET = 6;
}

public static class rotationConstants {
  public static final double ENCODER_DEGREES_PER_ROTATION = 360.0; // Adjust based on your encoder's resolution and gearing
}

  public static class ShooterConstants {
  public static final double kShooterSpeed = .9;
  public static final double kShooterSpeedReverse = 0.1;
  }
  public static class AcceleratorConstants
  {
    public static final int kAcceleratorMotorId = 23;
    public static final double kAcceleratorSpeed = 1;
  }

  public static class SpindexerConstants
  {
    public static final int kSpindexerMotorId = 22;
    public static final double kSpindexerSpeed = 0.8;
    public static final double kSpindexerSpeedReverse = 0.2;

  }

  public static class IntakeConstants
  {
    public static final int kUpDown_MotorId = 20;
    public static final int kRoller_MotorId = 21;
    public static final double kRollerMotorSpeed = 0.5;
    public static final double kUpDownMotorSpeed = 0.25;

  }
}