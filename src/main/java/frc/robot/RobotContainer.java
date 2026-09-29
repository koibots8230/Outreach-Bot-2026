package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ShootCommands;
import frc.robot.subsystems.Shooter;

@Logged
public class RobotContainer {
  @NotLogged private final XboxController controller;
  
  private final Shooter shooter;

  public RobotContainer(boolean isReal) {
    controller = new XboxController(0);

    shooter = new Shooter();

    configureBindings();
  }

  private void configureBindings() {
    Trigger shootHigh = new Trigger(controller::getAButton); //trigger buttons not set in stone
    shootHigh.onTrue(ShootCommands.shootHigh(shooter));
    shootHigh.onFalse(ShootCommands.stop(shooter));

    Trigger shootLow = new Trigger(controller::getYButton);
    shootLow.onTrue(ShootCommands.shootLow(shooter));
    shootLow.onFalse(ShootCommands.stop(shooter));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
