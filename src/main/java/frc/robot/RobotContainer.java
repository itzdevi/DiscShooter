package frc.robot;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.FidingSubsistem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.UpAndDown;

public class RobotContainer implements Sendable{

  public Shooter shooter = new Shooter();
  public Chassis chassis = new Chassis();
  public FidingSubsistem fid = new FidingSubsistem();
  public UpAndDown uad = new UpAndDown();
  public CommandXboxController controler = new CommandXboxController(0);
  
  public RobotContainer() {
    SmartDashboard.putData("RC", this);
    configureBindings();

    
    
  }

  @Override
  public void initSendable(SendableBuilder builder) {

    builder.addDoubleProperty("v", shooter::getShootingVelocity, null);
    builder.addDoubleProperty("p", shooter::getShootingPower, null);
    builder.addDoubleProperty("c", fid::getMFidVoltage, null);
  }
  private void configureBindings() {
    
  }
  public Command getAutonomousCommand() {
    return null;
  }
  
}
