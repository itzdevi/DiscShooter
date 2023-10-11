package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Shooter;

import static frc.robot.Constants.ShooterConstants.*;

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
    shooter.resetHood();
  }

  @Override
  public void execute() {
    boolean a = controller.a().getAsBoolean();
    double leftY = controller.getLeftY();
    shooter.setShootingVelocity((a ? 1 : 0) * FLYWHEEL_VELOCITY);
    shooter.setHoodVelocity(leftY * HOOD_VELOCITY);
  }
}
