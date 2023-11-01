package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ManualShootCommand;
import frc.robot.subsystems.Shooter;

public class RobotContainer {
  Shooter shooter;
  CommandXboxController controler;
  ManualShootCommand mshoot;

  public RobotContainer() {
    shooter = new Shooter();
    controler = new CommandXboxController(0);
    mshoot = new ManualShootCommand(shooter, controler);
  }
  
  public Command getAutonomousCommand() {
    return mshoot;
  }
}
