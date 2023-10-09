// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

// Subsystem for the shooter mechanism
public class shooter extends SubsystemBase {
  private final TalonFX flywheel;
  private final TalonFXConfiguration fconfig;
  private final PIDController fpidController;
  private final SimpleMotorFeedforward ffeedforward;
  private final TalonSRX hood;
  private final TalonSRXConfiguration hconfig;
  private final PIDController hpidController;
  private final SimpleMotorFeedforward hfeedforward;
  // Network table for Limelight, the vision system
  private final NetworkTable limelightTable;

  // Constructor initializes subsystem components
  public shooter() {
    limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    flywheel = new TalonFX(Constants.flywheel); 
    fconfig = new TalonFXConfiguration();
    fconfig.supplyCurrLimit.enable = true;
    fconfig.supplyCurrLimit.triggerThresholdCurrent = 40; 
    fconfig.supplyCurrLimit.triggerThresholdTime = 1.5; 
    fconfig.supplyCurrLimit.currentLimit = 30;
    // Applying current limit settings to the flywheel
    flywheel.configAllSettings(fconfig);
    fpidController = new PIDController(Constants.fkP, 0.0, 0.0);
    ffeedforward = new SimpleMotorFeedforward(Constants.fkS, Constants.fkV, Constants.fkA);
    hood = new TalonSRX(Constants.hood); // Corrected the constant to hood instead of flywheel 
    hconfig = new TalonSRXConfiguration();
    // Applying default settings to the TalonSRX
    hood.configAllSettings(hconfig);
    hpidController = new PIDController(Constants.hkP, Constants.hkI, Constants.hkD);
    hfeedforward = new SimpleMotorFeedforward(Constants.hkS, Constants.hkV, Constants.hkA);
  }

  // Methods to get Limelight data
  public double getTargetX() {
    return limelightTable.getEntry("tx").getDouble(0.0);
  }

  public double getTargetY() {
    return limelightTable.getEntry("ty").getDouble(0.0);
  }

  public double getTargetArea() {
    return limelightTable.getEntry("ta").getDouble(0.0);
  }

  // Control loop for adjusting wheel velocity
  public void setTargetVelocity(double targetVelocity) {
    double pidOutput = fpidController.calculate(getSensorVelocity(), targetVelocity);
    double feedforwardOutput = ffeedforward.calculate(targetVelocity);

    double output = pidOutput + feedforwardOutput;
    output = MathUtil.clamp(output, -1.0, 1.0);
    this.setPower(output);
  }

  // setTalonSRXPositionToZero() resets the encoder value to 0
  public void setTalonSRXPositionToZero() {
    hood.setSelectedSensorPosition(0);
  }

  // Control hood position with PID
  public void sethoodtoposition(double target) {
    double pidOutput = hpidController.calculate(hood.getSelectedSensorPosition(), target);
    double feedforwardOutput = hfeedforward.calculate(target);

    double output = pidOutput + feedforwardOutput;
    output = MathUtil.clamp(output, -1.0, 1.0);
    hood.set(TalonSRXControlMode.PercentOutput, output);
  }

  // Controls flywheel power
  public void setPower(double power) {
    flywheel.set(TalonFXControlMode.PercentOutput, power);
  }

  // Returns sensor velocity for PID calculation
  public double getSensorVelocity() {
    return flywheel.getSelectedSensorVelocity();
  }

}