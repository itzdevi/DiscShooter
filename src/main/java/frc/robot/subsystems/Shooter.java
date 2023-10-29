// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import static frc.robot.Constants.ShooterConstants.*;

public class Shooter extends SubsystemBase {
  private final TalonFX mFlywheel;
  private final TalonSRX mHood;
  private final CANCoder hoodAbsoulteEncoder;
  
  private final SimpleMotorFeedforward flywheelFF;
  private final SimpleMotorFeedforward hoodFF;

  private final NetworkTable limelightTable;

  private double hoodAngleOffset;

  // Constructor initializes subsystem components
  public Shooter() {
    mFlywheel = new TalonFX(FLYWHEEL_MOTOR_ID);
    mHood = new TalonSRX(HOOD_MOTOR_ID);
    hoodAbsoulteEncoder = new CANCoder(HOOD_ABSOULTE_ENCODER_ID);

    flywheelFF = new SimpleMotorFeedforward(FLYWHEEL_KS, FLYWHEEL_KD, FLYWHEEL_KA);
    hoodFF = new SimpleMotorFeedforward(HOOD_KS, HOOD_KD, HOOD_KA);

    limelightTable = NetworkTableInstance.getDefault().getTable(LIMELIGHT_TABLE_KEY);

    setFlywheelPID(0.5, 0, 0);
    setHoodlPID(0.5, 0, 0);
    calibrateHoodOffset();
  }

  // Methods to get Limelight data
  public double getTargetX() {
    return limelightTable.getEntry("wdist").getDouble(0.0);
  }

  public double getTargetY() {
    return limelightTable.getEntry("hdist").getDouble(0.0);
  }
  
  public double getTargetArea() {
    return limelightTable.getEntry("dist").getDouble(0.0);
  }
  public double getTargetXAngle() {
    return limelightTable.getEntry("wangle").getDouble(0.0);
  }

  public double getTargetYAngle() {
    return limelightTable.getEntry("hangle").getDouble(0.0);
  }

  public Translation2d getTargetVector() {
    return new Translation2d(getTargetArea(), getTargetY());
  }
  
  public void setFlywheelPID(double kP, double kI, double kD) {
    mFlywheel.config_kP(0, kP);
    mFlywheel.config_kI(0, kI);
    mFlywheel.config_kD(0, kD);
  }

  public void setHoodlPID(double kP, double kI, double kD) {
    mHood.config_kP(0, kP);
    mHood.config_kI(0, kI);
    mHood.config_kD(0, kD);
  }
  
  /**
   * Sets the velocity of the flywheel
   * @param targetVelocity Velocity in m/s
   */
  public void setShootingVelocity(double targetVelocity) {
    double volts = flywheelFF.calculate(getShootingVelocity(), targetVelocity, Constants.CYCLE_DT);
    mFlywheel.set(
      ControlMode.Velocity, targetVelocity * FLYWHEEL_PULSES_PER_METER * 0.1,
      DemandType.ArbitraryFeedForward, volts / 12
    );
  }

  /**
   * Returns the velocity of the flywheel
   * @return Velocity in m/s
   */
  public double getShootingVelocity() {
    return mFlywheel.getSelectedSensorVelocity() / FLYWHEEL_PULSES_PER_METER * 10;
  }

  public void stopHood() {
    mHood.set(ControlMode.PercentOutput, 0);
  }

  public void setHoodPower(double power) {
    mHood.set(ControlMode.PercentOutput, power);
  }

  /**
   * Sets the velocity of the hood
   * @param targetVelocity Velocity in m/s
   */
  public void setHoodVelocity(double targetVelocity) {
    double volts = hoodFF.calculate(getHoodVelocity(), targetVelocity, Constants.CYCLE_DT);
    mHood.set(
      ControlMode.Velocity, targetVelocity * HOOD_PULSES_PER_METER * 0.1,
      DemandType.ArbitraryFeedForward, volts / 12
    );
  }

  /**
   * Returns the velocity of the hood
   * @return Velocity in m/s
   */
  public double getHoodVelocity() {
    return mHood.getSelectedSensorVelocity() / HOOD_PULSES_PER_METER * 10;
  }

  public void setHoodAngle(double angle) {
    double volts = hoodFF.calculate(HOOD_VELOCITY, HOOD_ACCELERATION);
    mHood.set(ControlMode.Position, calculateTargetPosition(angle), DemandType.ArbitraryFeedForward, volts / 12);
  }

  public void resetHood() {
    setHoodAngle(0);
  }
  
  public void setShootingPower(double power) {
    mFlywheel.set(ControlMode.PercentOutput, power);
  }

  public void stopShooting() {
    setShootingPower(0);
  }

  public void calibrateHoodOffset() {
    hoodAngleOffset = hoodAbsoulteEncoder.getAbsolutePosition();
  }

  public Rotation2d getHoodAngle() {
    return Rotation2d.fromDegrees(hoodAbsoulteEncoder.getAbsolutePosition() + hoodAngleOffset);
  }

  private double calculateTargetPosition(double targetAngle) {
    double difference = getAngleDifference(getHoodAngle().getDegrees(), targetAngle);
    return mHood.getSelectedSensorPosition() + (difference * HOOD_PULSES_PER_DEGREE);
  }

  public static double getAngleDifference(double current, double target) {
    double difference = (target - current) % 360;
    return difference - ((int)difference / 180) * 360;
  }
}