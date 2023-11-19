// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.UpAndDown;
import frc.robot.Trapez;

public class UADCommend extends CommandBase {
  private UpAndDown UAD;
  private double goToPich;
  private double startPich;
  private double remainingPich;
  private Trapez Trapez;
  public UADCommend(UpAndDown UAD, double goToPich) {
    this.UAD = UAD;
    this.goToPich = goToPich;
    Trapez = new Trapez(0.2, 0.7);
    addRequirements(UAD);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startPich = UAD.getpigonPitch();  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    remainingPich = (goToPich + startPich) - UAD.getpigonPitch();
    double velocity = Trapez.calculate(remainingPich, UAD.getpigonPitch(), 0.4);
    UAD.SetUADVelosyty(velocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
