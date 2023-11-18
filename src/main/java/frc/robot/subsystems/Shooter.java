// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ShooterConstants.*;

public class Shooter extends SubsystemBase {
  private TalonFX mFlywheel;

  public Shooter() {
    super();
    mFlywheel = new TalonFX(FLYWHEEL_MOTOR_ID);

    setPID(mFlywheel, 0.7, 0, 0);
    
    mFlywheel.setInverted(false);
  }

  public void setPID(TalonFX motor, double kP, double kI, double kD) {
    motor.config_kP(0, kP);
    motor.config_kP(0, kI);
    motor.config_kP(0, kD);
  }

  public double getShootingVelocity() {
    return -mFlywheel.getSelectedSensorVelocity();
  }

  public void setShootingPower(double power) {
    mFlywheel.set(ControlMode.PercentOutput, power);
  }
  
  public void setShootingVelosity(double v) {
    mFlywheel.set(ControlMode.Velocity, v*PULSES_PER_METER/10, DemandType.ArbitraryFeedForward, calcFF(v));
}
  public double getShootingPower() {
    return-(mFlywheel.getMotorOutputPercent());
  }

  public void stopShooting() {
    setShootingPower(0);
  }

  public double calcFF(double v) {
    return ks + kv*v + kv2*v*v;
  }
  
}
