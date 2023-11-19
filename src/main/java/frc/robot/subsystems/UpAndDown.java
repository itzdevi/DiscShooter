// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ShooterConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;
public class UpAndDown extends SubsystemBase {
  /** Creates a new FidingSubsistem. */
  private TalonFX mUAD;
  private PigeonIMU gyro;
  public UpAndDown() {

    mUAD = new TalonFX(FID_MOTOR_ID);
    setPID(mUAD, 0.4, 0.04, 0.004);

    gyro = new PigeonIMU(GYRO_ID);
  }

  public void setPID(TalonFX motor, double kP, double kI, double kD) {
    motor.config_kP(0, kP);
    motor.config_kP(0, kI);
    motor.config_kP(0, kD);
  }

  public void SetUADVelosyty(double UADVelosyty){
    mUAD.set(ControlMode.Velocity,UADVelosyty);
    
  }

  public double getMFidVoltage() {
    return mUAD.getMotorOutputVoltage();
  }
  
  public double getpigonPitch(){
    return gyro.getPitch();
  }
}
