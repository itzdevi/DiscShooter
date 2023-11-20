// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// Import necessary libraries
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ShooterConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

// Define FidingSubsistem class
public class FidingSubsistem extends SubsystemBase {
  // Create private instance of TalonFX
  private TalonFX mFid;

  // Default Constructor for FidingSubsistem class
  public FidingSubsistem() {
    // Initialize mFid with FID_MOTOR_ID
    mFid = new TalonFX(FID_MOTOR_ID);
    // Set the PID of mFid
    setPID(mFid, 0.4, 0.04, 0.004);
  }

  // Method to set PID for a motor
  public void setPID(TalonFX motor, double kP, double kI, double kD) {
    // Configure settings of the motor
    motor.config_kP(0, kP);
    motor.config_kP(0, kI);
    motor.config_kP(0, kD);
  }

  // Method to control Fid speed
  public void Fid(double fidSpeed){
    // Control motor speed until it reaches normal voltage
    while(mFid.getMotorOutputVoltage()>NORMAL_VOLTEG_OF_SNOW_BLOWER){
      mFid.set(ControlMode.PercentOutput,fidSpeed);
    }
  }

  // Method to set the power of mFid
  public void setMFidPower(double fidPower){
    mFid.set(ControlMode.PercentOutput,fidPower);
  }

  // Method to get voltage of mFid
  public double getMFidVoltage() {
    return mFid.getMotorOutputVoltage();
  }
}