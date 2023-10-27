package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ManualShootCommand;
import frc.robot.subsystems.Shooter;

public class RobotContainer {
  Shooter shooter = new Shooter();
  CommandXboxController controler = new CommandXboxController(0);
  ManualShootCommand mshoot = new ManualShootCommand(shooter, controler);

  public RobotContainer() {
  }
  
  public Command getAutonomousCommand() {
    return mshoot;
  }
}
