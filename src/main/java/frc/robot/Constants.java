package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

// Class holding all the constants
public final class Constants {
  public static final double CYCLE_DT = 0.02;  // Cycle DT constant

  // Nested class for Shooter constants
  public static class ShooterConstants {
    public static final int FLYWHEEL_MOTOR_ID = 1;  // ID of the flywheel motor
    public static final String LIMELIGHT_TABLE_KEY = "limelight-iii";  // Key for limelight table

    public static final double FLYWHEEL_VELOCITY_Multiplayer = 0.1;  // Velocity multiplier for the flywheel

    public static final int GYRO_ID = 14;  // ID of the Gyro

    public static final int FID_MOTOR_ID = 6;  // ID of the Snow blower

    public static final double NORMAL_VOLTEG_OF_SNOW_BLOWER = 0.5;  // ID of the Snow blower

    // IDs of front and back motors for left and right
    public static final int LeftFrontMotor = 2;
    public static final int LeftBackMotor = 3;
    public static final int RightFrontMotor = 4;
    public static final int RightBackMotor = 5;

    // Constants to set left and right inverted
    public static final boolean LeftInverted = true;
    public static final boolean RightInverted = false;

    // Constant for Wheel Circumference, Gear Ratio, Pulses per meter and rotation
    public static final double WheelCircumference = 4 * 0.0254 * Math.PI; // 4 Inch wheels
    public static final double GearRatio = 8.14;
    public static final double PulsePerRotation = 2048;
    public static final double PULSES_PER_METER = (1/WheelCircumference)*GearRatio*PulsePerRotation;

    public static final double ks = 0.05;
    public static final double kv = 0.01;
    public static final double kv2 = 0.0001;

    public static DifferentialDriveKinematics KINEMATICS = new DifferentialDriveKinematics(1);
  }

}
