package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Shooter;

public class ManualShootCommand extends CommandBase {
  private Shooter shooter;
  private CommandXboxController controller;


  public ManualShootCommand(Shooter shooter, CommandXboxController controller) {
    this.shooter = shooter;
    this.controller = controller;

    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    shooter.stopShooting();
  }

  @Override
  public void execute() {
    shooter.setShootingPower(controller.getLeftY());
    double vx = 0;
    double vOmega = controller.getRightX();
    ChassisSpeeds speeds = new ChassisSpeeds(vx, 0, vOmega);
    shooter.setVelocity(speeds);

  }
}
