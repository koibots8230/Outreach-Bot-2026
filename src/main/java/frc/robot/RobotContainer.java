package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.*;

@Logged
public class RobotContainer {
  private final Swerve swerve;
  @NotLogged private boolean isBlue;
  @NotLogged private Alliance alliance;
  private final CommandXboxController xboxController;

  public RobotContainer() {
    swerve = new Swerve();
    xboxController = new CommandXboxController(RobotConstants.CONTROLLER_PORT);
    configureBindings();
    setIsBlue();
    defaultCommands();
  }

  private void configureBindings() {}

  private void defaultCommands() {
    swerve.setDefaultCommand(
        swerve.driveCommand(
            () -> xboxController.getLeftY(),
            () -> xboxController.getLeftX(),
            () -> xboxController.getRightX()));
  }

  public void setIsBlue() {
    swerve.setIsBlue(DriverStation.getAlliance().map(a -> a == Alliance.Blue).orElse(false));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
