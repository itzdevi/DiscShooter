// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import static frc.robot.Constants.ShooterConstants.*;

public class Shooter extends SubsystemBase {
  private TalonFX mFlywheel;

  private TalonFX left;
  private TalonFX right;

  private TalonFX mFid;
  private PigeonIMU gyro;
  private Field2d field;
  DifferentialDriveOdometry odometry;
  private DifferentialDriveWheelSpeeds wheelSpeeds;
  // Constructor initializes subsystem components
  public Shooter() {
    super();
    mFlywheel = new TalonFX(FLYWHEEL_MOTOR_ID);

    mFid = new TalonFX(FID_MOTOR_ID);

    left = initMotors(LeftFrontMotor, LeftBackMotor, LeftInverted);
    right = initMotors(RightFrontMotor, RightBackMotor, RightInverted);

    setPID(mFlywheel, 0.7, 0, 0);
    setPID(mFid, 0.4, 0.04, 0.004);
    mFlywheel.setInverted(false);

    gyro = new PigeonIMU(GYRO_ID);
    field = new Field2d();
    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(0), 0, 0);
    
  }

  @Override
  public void periodic() {
    odometry.update(Rotation2d.fromDegrees(gyro.getFusedHeading()),
    left.getSelectedSensorPosition()/PULSES_PER_METER,
    right.getSelectedSensorPosition()/PULSES_PER_METER);
    field.setRobotPose(odometry.getPoseMeters());
  }

  private TalonFX initMotors(int main, int follower, boolean invert) {
    TalonFX m = new TalonFX(main);
    TalonFX f = new TalonFX(follower);
    m.setInverted(invert);
    f.setInverted(invert);
    f.follow(m);
    setPID(m, 0.3, 0.03,0.003);
    return m;
}
  
  public void setPID(TalonFX motor, double kP, double kI, double kD) {
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
  public void setShootingVelosity(double v) {
    mFlywheel.set(ControlMode.Velocity, v*PULSES_PER_METER/10, DemandType.ArbitraryFeedForward, calcFF(v));
}
  public double getShootingPower() {
    return-(mFlywheel.getMotorOutputPercent());
  }

  public void stopShooting() {
    setShootingPower(0);
  }


  public void setVelocity(ChassisSpeeds chassisSpeeds){
    wheelSpeeds = KINEMATICS.toWheelSpeeds(chassisSpeeds);
    left.set(ControlMode.Velocity, wheelSpeeds.leftMetersPerSecond*PULSES_PER_METER);
    right.set(ControlMode.Velocity, wheelSpeeds.rightMetersPerSecond*PULSES_PER_METER);
  }
  public double Angle(){
    return gyro.getFusedHeading();
  }
  public double calcFF(double v) {
    return ks + kv*v + kv2*v*v;
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
//getOutputCurrent