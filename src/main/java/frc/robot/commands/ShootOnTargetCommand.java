package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Shooter;

import static frc.robot.Constants.ShooterConstants.*;

public class ShootOnTargetCommand extends CommandBase {
  private final Shooter shooter;
  private final CommandXboxController controller;

  public ShootOnTargetCommand(Shooter shooter, CommandXboxController controller) {
    this.shooter = shooter;
    this.controller = controller;
  }

  @Override
  public void initialize() {
      shooter.stopShooting();
      shooter.stopHood();
  }

  @Override
  public void execute() {
    boolean a = controller.a().getAsBoolean();
    shooter.setShootingVelocity(a ? 1 : 0 * FLYWHEEL_VELOCITY);

    Rotation2d angleToTarget = shooter.getTargetVector().getAngle();
    shooter.setHoodAngle(angleToTarget.getDegrees());

    System.out.println(shooter.getTargetX() > 1 ? "rotate left" : "rotate right");
  }
}
