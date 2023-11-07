package frc.robot;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ManualShootCommand;
import frc.robot.subsystems.Shooter;

public class RobotContainer implements Sendable{

  public Shooter shooter = new Shooter();
  public CommandXboxController controler = new CommandXboxController(0);
  public ManualShootCommand command = new ManualShootCommand(shooter, controler);
  
  public RobotContainer() {
    SmartDashboard.putData("RC", this);
    configureBindings();
    shooter.setDefaultCommand(command);
    
    
  }

  @Override
  public void initSendable(SendableBuilder builder) {

    builder.addDoubleProperty("v", shooter::getShootingVelocity, null);
    builder.addDoubleProperty("p", shooter::getShootingPower, null);
  }
  private void configureBindings() {
    
  }
  public Command getAutonomousCommand() {
    return null;
  }
  
}
