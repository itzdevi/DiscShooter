package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class ManualShootCommand extends CommandBase {
  private Shooter shooter;
  private Double dist;

  public ManualShootCommand(Shooter shooter,Double dist) {
    this.shooter = shooter;
    this.dist = dist;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    shooter.stopShooting();
  }

  @Override
  public void execute() {
    shooter.setShootingPower(dist);

  }
}
