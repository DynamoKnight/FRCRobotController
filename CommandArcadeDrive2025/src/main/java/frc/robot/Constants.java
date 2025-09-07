// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class Operator {
    public static final int CONTROLLER_PORT_0 = 0;
    public static final int CONTROLLER_PORT_1 = 1;
  }

  public static class Drivetrain {
    public static final int FRONT_LEFT_ID = 1;
    public static final int BACK_LEFT_ID = 2;
    public static final int FRONT_RIGHT_ID = 3;
    public static final int BACK_RIGHT_ID = 4;
    public static final double SLOW_MOVE_POWER = 0.3;
    public static final double SLOW_TURN_POWER = 0.3;
    public static final double FAST_MOVE_POWER = 1;
    public static final double FAST_TURN_POWER = 0.7;
    public static final double WHEEL_DIAMETER = 0.1524; // 6 in
    public static final double GEAR_RATIO = 10.71;
    public static final double ENCODERS_TO_METERS = WHEEL_DIAMETER * Math.PI / GEAR_RATIO;
    public static final double TRACK_WIDTH_METERS = 0.5; // Distance between the left and right wheels

  }

}
