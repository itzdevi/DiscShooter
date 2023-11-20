// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

// Importing required subsystems and classes
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.UpAndDown;
import frc.robot.Trapez;

// Class UADCommend inherits CommandBase
public class UADCommend extends CommandBase {

  // Declaring private variables and object
  private UpAndDown UAD;
  private double goToPich;
  private double startPich;
  private double remainingPich;
  private Trapez Trapez;

  // Constructor for the UADCommend class
  public UADCommend(UpAndDown UAD, double goToPich) {
    this.UAD = UAD;
    this.goToPich = goToPich;
    Trapez = new Trapez(0.2, 0.7);  // Initializing the Trapez object
    addRequirements(UAD);  // Adding subsystem requirement
  }

  // This is called when the command is initially scheduled.
  @Override
  public void initialize() {
    startPich = UAD.getpigonPitch();  // Gets the initial pitch  
  }

  // This is called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Calculate remainingPich value
    remainingPich = (goToPich + startPich) - UAD.getpigonPitch();

    // Calculate velocity and set the subsystem's velocity using it
    double velocity = Trapez.calculate(remainingPich, UAD.getpigonPitch(), 0.4);
    UAD.SetUADVelosyty(velocity);
  }

  // This is called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}