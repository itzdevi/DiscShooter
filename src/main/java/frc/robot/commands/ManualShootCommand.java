package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Shooter;


public class ManualShootCommand extends CommandBase {
  private final Shooter shooter;
  private final CommandXboxController controller;
  

  public ManualShootCommand(Shooter shooter, CommandXboxController controller) {
    this.shooter = shooter;
    this.controller = controller;

    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    shooter.stopShooting();
    shooter.setServoToAngle(0);
  }

  @Override
  public void execute() {
    shooter.setShootingPower(controller.getLeftY());
    shooter.setServoToAngle(90);

  }
}
