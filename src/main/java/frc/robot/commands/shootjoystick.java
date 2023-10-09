// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter;

public class shootjoystick extends CommandBase {
  private shooter shoot;
  private XboxController controller;
  private double maxhood;
  /** Creates a new shootjoystick. */
  public shootjoystick(shooter shoot, XboxController controller,double maxhood) {
    this.shoot = shoot;
    this.controller = controller;
    this.maxhood = maxhood;
    addRequirements(shoot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shoot.setTalonSRXPositionToZero();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      shoot.setPower(controller.getRawAxis(XboxController.Axis.kRightY.value));
      if (controller.getRawButton(XboxController.Button.kY.value)) {
          shoot.sethoodtoposition(0);
      } else if (controller.getRawButton(XboxController.Button.kB.value)) {
          shoot.sethoodtoposition(maxhood/3);
      } else if (controller.getRawButton(XboxController.Button.kA.value)) {
          shoot.sethoodtoposition(maxhood-(maxhood/3));
      } else if (controller.getRawButton(XboxController.Button.kX.value)) {
          shoot.sethoodtoposition(maxhood);
      }
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
