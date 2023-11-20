package frc.robot.commands;

// Importing necessary libraries
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

// ManualShootCommand class inherits CommandBase
public class ManualShootCommand extends CommandBase {
  // Declaring private variables for Shooter and distance
  private Shooter shooter;
  private Double dist;

  // ManualShootCommand constructor with Shooter and distance as parameters
  public ManualShootCommand(Shooter shooter,Double dist) {
    // Assigning parameter values to private variables
    this.shooter = shooter;
    this.dist = dist;
    // Adding requirements
    addRequirements(shooter);
  }

  // Overriding the initialize method in parent class
  @Override
  public void initialize() {
    // Stops shooting
    shooter.stopShooting();
  }

  // Overriding the execute method in parent class
  @Override
  public void execute() {
    // Set shooting power with the passed distance
    shooter.setShootingPower(dist);

  }
}