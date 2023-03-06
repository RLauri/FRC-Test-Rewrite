// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;
import swervelib.parser.PIDFConfig;

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

  // TODO: Ask Matt for values
  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag

  public static final class Auton
  {

    public static final PIDFConfig xAutoPID     = new PIDFConfig(0.7, 0, 0);
    public static final PIDFConfig yAutoPID     = new PIDFConfig(0.7, 0, 0);
    public static final PIDFConfig angleAutoPID = new PIDFConfig(0.4, 0, 0.01);

    // these values appear to be no longer used, max acceleration is calculated using wheel grip coeff and speed is set in jsons
    // public static final double MAX_SPEED        = 14.5;
    // public static final double MAX_ACCELERATION = 10;
  }

  public static final class Drivebase
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    // set to Ryan's 7-year old joystick values 
    public static final double LEFT_X_DEADBAND = 0.05;
    public static final double LEFT_Y_DEADBAND = 0.05;

    // Driver controller port
    public static final int DRIVER_PORT = 0;

    // Appendage controller port
    public static final int APPENDAGE_PORT = 1;
  }

  public static class IDs
  {
    // pneumatic hub
    public static final int PNEUMATIC_CAN_ID = 19;
    // claw solenoid ports 
    public static final int CLAW_OPEN_PORT = 0;
    public static final int CLAW_CLOSE_PORT = 1;

    // arm extension motor controller id
    public static final int EXTENSION_CAN_ID = 4;
    public static final int EXTENSION_ENC_ID = 0;

    // TODO: correct
    // power distribution hub
    public static final int PDH_CAN_ID = 42;

  }

  public static class Extension 
  {
    public static final double MIN_POS = 0.5;
    public static final double MAX_POS = 4.8;

    public static final double ZEROING_CUR = 0.2;

    public static final double kP = .5;
    public static final double kI = 0;
    public static final double kD = .2;
    public static final double kDt = 0.02;
    public static final double kMaxVelocityRadPerSecond = 3;
    public static final double kMaxAccelerationRadPerSecSquared = 5;
    public static final double kPosTolerance = 0.05;

  }
}
