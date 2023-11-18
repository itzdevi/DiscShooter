package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;
import frc.robot.Trapez;

public class GoToShotingRotation extends CommandBase {
  private Chassis chassis;
  private double xangl;
  private Trapez Trapez;
  private double startangle;
  private double remainingangle;

  public GoToShotingRotation(Chassis chassis, double xangl) {
    this.chassis = chassis;
    this.xangl = xangl;
    Trapez = new Trapez(0.2, 0.7);
    addRequirements(chassis);
  }

  @Override
  public void initialize() {
    startangle = chassis.Angle();
  }

  @Override
  public void execute() {
    remainingangle = (xangl + startangle) - chassis.Angle();
    double vx = 0;
    double vOmega = Trapez.calculate(remainingangle, chassis.Angle(), 0.5);
    ChassisSpeeds speeds = new ChassisSpeeds(vx, 0, vOmega);
    chassis.setVelocity(speeds);

  }
}
