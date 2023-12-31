// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FidingSubsistem;

public class FidDisck extends CommandBase {
  FidingSubsistem fid;
  public FidDisck(FidingSubsistem fid) {
    this.fid = fid;

    addRequirements(fid);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    fid.setMFidPower(0);
    for(int i=0; i >= 4; i++){
      fid.Fid(0.6);
      fid.Fid(-0.6);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    fid.Fid(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
