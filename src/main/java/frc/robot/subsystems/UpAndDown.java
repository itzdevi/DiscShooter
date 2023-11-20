// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ShooterConstants.*;

// Utilities to control the motor and obtain sensor readings
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;

public class UpAndDown extends SubsystemBase {

  // Motor instance for the UpAndDown subsystem
  private TalonFX mUAD;

  // Inertial measurement unit (IMU)
  private PigeonIMU gyro;

  public UpAndDown() {

    // Initialize the motor with its unique ID
    mUAD = new TalonFX(FID_MOTOR_ID);

    // Set PID constants for motor control
    setPID(mUAD, 0.4, 0.04, 0.004);

    // Initialize the IMU with its unique ID
    gyro = new PigeonIMU(GYRO_ID);
  }

  // Method to set the PID coefficients
  public void setPID(TalonFX motor, double kP, double kI, double kD) {
    motor.config_kP(0, kP);
    motor.config_kP(0, kI);
    motor.config_kP(0, kD);
  }

  // Method to set the velocity of the motor
  public void SetUADVelosyty(double UADVelosyty){
    mUAD.set(ControlMode.Velocity,UADVelosyty);

  }

  // Method to get motor output voltage
  public double getMFidVoltage() {
    return mUAD.getMotorOutputVoltage();
  }

  // Method to get pitch angle from IMU
  public double getpigonPitch(){
    return gyro.getPitch();
  }
}