// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ShooterConstants.*;

// Control motor and sensor imports
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;

public class Chassis extends SubsystemBase {
  // Defines variables for subsequent use
  private TalonFX left;
  private TalonFX right;
  private PigeonIMU gyro;
  private Field2d field;
  private DifferentialDriveOdometry odometry;
  private DifferentialDriveWheelSpeeds wheelSpeeds;

  public Chassis() {
    // Initialises motors, gyro, field and odometry
    left = initMotors(LeftFrontMotor, LeftBackMotor, LeftInverted);
    right = initMotors(RightFrontMotor, RightBackMotor, RightInverted);

    gyro = new PigeonIMU(GYRO_ID);
    field = new Field2d();
    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(0), 0, 0);
  }

  @Override
  // Main loop, updates every so often
  public void periodic() {
    // Updates odometry with current position and heading
    odometry.update(Rotation2d.fromDegrees(gyro.getFusedHeading()),
    left.getSelectedSensorPosition()/PULSES_PER_METER,
    right.getSelectedSensorPosition()/PULSES_PER_METER);

    // Sets robot pose on the field
    field.setRobotPose(odometry.getPoseMeters());
  }

  // Initialises motors with given ID and inversion parameters, sets them up for PID
  private TalonFX initMotors(int main, int follower, boolean invert) {
    TalonFX m = new TalonFX(main);
    TalonFX f = new TalonFX(follower);
    m.setInverted(invert);
    f.setInverted(invert);
    f.follow(m);
    setPID(m, 0.3, 0.03,0.003);
    return m;
  }

  // Assigns PID values to a specific motor
  public void setPID(TalonFX motor, double kP, double kI, double kD) {
    motor.config_kP(0, kP);
    motor.config_kP(0, kI);
    motor.config_kP(0, kD);
  }

  // Sets the velocity of the chassis according to given speeds
  public void setVelocity(ChassisSpeeds chassisSpeeds){
    // Converts chassis speeds to left and right wheel speeds
    wheelSpeeds = KINEMATICS.toWheelSpeeds(chassisSpeeds);
    // Sets motor velocity to move at desired speeds
    left.set(ControlMode.Velocity, wheelSpeeds.leftMetersPerSecond*PULSES_PER_METER);
    right.set(ControlMode.Velocity, wheelSpeeds.rightMetersPerSecond*PULSES_PER_METER);
  }

  // Get the angle of the gyro
  public double Angle(){
    return gyro.getFusedHeading();
  }

}