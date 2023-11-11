package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Shooter;
import frc.robot.Trapez;

public class ManualShootCommand extends CommandBase {
  private Shooter shooter;
  private CommandXboxController controller;
  private double yangl;
  private double xangl;
  private double dist;
  private Trapez Trapez;
  private double startangle;
  private double remainingangle;

  public ManualShootCommand(Shooter shooter, CommandXboxController controller, double yangl, double xangl, double dist) {
    this.shooter = shooter;
    this.controller = controller;
    this.yangl = yangl;
    this.xangl = xangl;
    this.dist = dist;
    Trapez = new Trapez(0.2, 0.7);
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    shooter.stopShooting();
    shooter.setServoToAngle(0);
    startangle = shooter.Angle();
  }

  @Override
  public void execute() {
    shooter.setShootingPower(controller.getLeftY());
    shooter.setServoToAngle(90);
    remainingangle = (xangl + startangle) - shooter.Angle();
    double vx = 0;
    double vOmega = Trapez.calculate(remainingangle, shooter.Angle(), 0.5);
    ChassisSpeeds speeds = new ChassisSpeeds(vx, 0, vOmega);
    shooter.setVelocity(speeds);

  }
}
