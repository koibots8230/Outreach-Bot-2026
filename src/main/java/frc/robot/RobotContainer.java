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
    Trigger shoot = new Trigger(controller::getAButton); //trigger buttons not set in stone
    shoot.onTrue(ShootCommands.shoot(shooter));
    shoot.onFalse(ShootCommands.stop(shooter));

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
