// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ShooterConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
public class FidingSubsistem extends SubsystemBase {
  /** Creates a new FidingSubsistem. */
  private TalonFX mFid;
  public FidingSubsistem() {

    mFid = new TalonFX(FID_MOTOR_ID);
    setPID(mFid, 0.4, 0.04, 0.004);
  }

  public void setPID(TalonFX motor, double kP, double kI, double kD) {
    motor.config_kP(0, kP);
    motor.config_kP(0, kI);
    motor.config_kP(0, kD);
  }

  public void Fid(double fidSpeed){
    while(mFid.getMotorOutputVoltage()>NORMAL_VOLTEG_OF_SNOW_BLOWER){
      mFid.set(ControlMode.PercentOutput,fidSpeed);
    }
  }
  
  public void setMFidPower(double fidPower){
    mFid.set(ControlMode.PercentOutput,fidPower);
  }

  public double getMFidVoltage() {
    return mFid.getMotorOutputVoltage();
  }
}
