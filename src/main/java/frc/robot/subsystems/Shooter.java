// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// Importing necessary motor control and command classes
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Importing ShooterConstants from the Constants file
import static frc.robot.Constants.ShooterConstants.*;

// Defining a public class Shooter that extends Subsystem Base Class
public class Shooter extends SubsystemBase {

  // Declaring private TalonFX member variable mFlywheel
  private TalonFX mFlywheel;

  // Constructor for Shooter - implements the base class constructor and initializes mFlywheel 
  public Shooter() {
    super();
    mFlywheel = new TalonFX(FLYWHEEL_MOTOR_ID);

    setPID(mFlywheel, 0.7, 0, 0);

    mFlywheel.setInverted(false);
  }

  // Method to set the PID values for the motor control
  public void setPID(TalonFX motor, double kP, double kI, double kD) {
    motor.config_kP(0, kP);
    motor.config_kP(0, kI);
    motor.config_kP(0, kD);
  }

  // Getter for Shooting Velocity
  public double getShootingVelocity() {
    return -mFlywheel.getSelectedSensorVelocity();
  }

  // Setter for Shooting Power
  public void setShootingPower(double power) {
    mFlywheel.set(ControlMode.PercentOutput, power);
  }

  // Setter for Shooting Velocity
  public void setShootingVelosity(double v) {
    mFlywheel.set(ControlMode.Velocity, v*PULSES_PER_METER/10, DemandType.ArbitraryFeedForward, calcFF(v));
  }

  // Getter for Shooting Power
  public double getShootingPower() {
    return-(mFlywheel.getMotorOutputPercent());
  }

  // Method to stop shooting, essentially setting shooting power to zero
  public void stopShooting() {
    setShootingPower(0);
  }

  // Method to calculate feed forward for given velocity
  public double calcFF(double v) {
    return ks + kv*v + kv2*v*v;
  }

}
