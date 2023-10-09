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
  public static final int flywheel = 1;
  public static final double fkP = 0.7;
  public static final double fkV = 1/12;
  public static final double fkS = 2/12;
  public static final double fkA = 3/12;
  public static final int hood = 2;
  public static final double hkP = 0.2;
  public static final double hkI = 0.02;
  public static final double hkD = 0.002;
  public static final double hkV = 1/12;
  public static final double hkS = 2/12;
  public static final double hkA = 3/12;


}
