// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.PIDConstants;

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
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag

  public static final class AutonConstants
  {

    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
    public static final PIDConstants ANGLE_PID   = new PIDConstants(0.4, 0, 0.01);
  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double LEFT_X_DEADBAND  = 0.05;
    public static final double LEFT_Y_DEADBAND  = 0.05;
    public static final double ROTATE_DEADBAND = 0.15;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }

  public final class Feeder{
    public static final int outerMotorID = 31; //= 30;
    public static final int innerMotorID = 30; //= 31;
    public static final int positionMotorID = 32;

    public static final double outerMotorKP = 0.0002;
    public static final double outerMotorKI = 0.000001;
    public static final double outerMotorKD = 0.0;
    public static final double outerMotorMAX = 1.0;
    public static final double outerMotorMIN = -1.0;
    public static final int outerMotorCurrentLimit = 40;
    public static final int innerMotorCurrentLimit = 40;
    public static final double innerMotorKP = 0.0002;
    public static final double innerMotorKI = 0.000001;
    public static final double innerMotorKD = 0;
    public static final double innerMotorMIN = -1.0;
    public static final double innerMotorMAX = 1.0;
    public static final int positionMotorCurrentLimit = 20;
    public static final double positionMotorKP = 0.03;
    public static final double positionMotorKD = 0.0;
    public static final double positionMotorKI = 0;

    public static final int ampPosition = -180;
    public static final int shootPosition = -110;

    public static final double intakeSpeedFPS = .1;
    public static final double intakeSpeedOuter = -1500;
    public static final double intakeSpeedInner = -100;
    public static double intakeStopDistance;
    public static final double depositspeedFPS = 2;
    public static final double depositSpeed = -500;
    public static final int dropServo01Port = 0;
    public static final int dropServo02Port = 1;
    public static final int intakeSwitchPort = 0;
    public static final double shootSpeedInner = 4500;
    public static final double shootSpeedOuter = 4500;
}
}
