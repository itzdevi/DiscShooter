package frc.robot;

public final class Constants {
  public static final double CYCLE_DT = 0.02;

  public static class ShooterConstants {
    public static final int FLYWHEEL_MOTOR_ID = 1;
    public static final int HOOD_MOTOR_ID = 9;
    public static final String LIMELIGHT_TABLE_KEY = "limelight";

    public static final double FLYWHEEL_KS = 0;
    public static final double FLYWHEEL_KD = 0;
    public static final double FLYWHEEL_KA = 0;

    public static final double HOOD_KS = 0;
    public static final double HOOD_KD = 0;
    public static final double HOOD_KA = 0;

    public static final double FLYWHEEL_PULSES_PER_METER = 1;
    public static final double HOOD_PULSES_PER_METER = 1;
    public static final double HOOD_PULSES_PER_DEGREE = 1;

    public static final double FLYWHEEL_VELOCITY = 20;
    public static final double HOOD_VELOCITY = 0.7;
    public static final double HOOD_ACCELERATION = 2;
  }
}
