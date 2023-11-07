// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Servo;

import static frc.robot.Constants.ShooterConstants.*;

public class Shooter extends SubsystemBase {
  private final TalonFX mFlywheel;
  private final Servo servo;

  private final NetworkTable limelightTable;


  // Constructor initializes subsystem components
  public Shooter() {
    super();
    mFlywheel = new TalonFX(FLYWHEEL_MOTOR_ID);
    servo = new Servo(SERVO_ID);

    limelightTable = NetworkTableInstance.getDefault().getTable(LIMELIGHT_TABLE_KEY);

    setFlywheelPID(0.7, 0, 0);
    mFlywheel.setInverted(false);
    
  }

  // Methods to get Limelight data
  public double getTargetX() {
    return limelightTable.getEntry("tx").getDouble(0.0);
  }
  
  public void setFlywheelPID(double kP, double kI, double kD) {
    mFlywheel.config_kP(0, kP);
    mFlywheel.config_kP(0, kI);
    mFlywheel.config_kP(0, kD);
  }

  public double getShootingVelocity() {
    return -mFlywheel.getSelectedSensorVelocity();
  }

  public void setShootingPower(double power) {
    mFlywheel.set(ControlMode.PercentOutput, power);
  }
  public double getShootingPower() {
    return-(mFlywheel.getMotorOutputPercent());
  }

  public void stopShooting() {
    setShootingPower(0);
  }
  public void setServoToAngle(double servoAngle){
    servo.setAngle(servoAngle);
  }
  
  
}