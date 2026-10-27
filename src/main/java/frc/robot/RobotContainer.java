package frc.robot;


import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.RobotConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

@Logged
public class RobotContainer {
  @NotLogged private final XboxController xboxController;
  @NotLogged private final GenericHID operator;
  private final Swerve swerve;
  private final Spindexer spindexer;
  private final TowerIndexer towerIndexer;
  private final Shooter shooter;

  public RobotContainer() {
    xboxController = new XboxController(RobotConstants.CONTROLLER_PORT);
    operator = new GenericHID(1);
    spindexer = new Spindexer();
    towerIndexer = new TowerIndexer();
    shooter = new Shooter();
    swerve = new Swerve();
    ;
    configureBindings();
    setIsBlue();
    defaultCommands();
  }

  private void configureBindings() {
    Trigger intakeButton = new Trigger(() -> xboxController.getLeftTriggerAxis() > 0.15);
    intakeButton.onTrue(IntakeCommands.startIntake(spindexer));
    intakeButton.onFalse(IntakeCommands.stopIntake(spindexer));

    Trigger intakeReverse = new Trigger(() -> xboxController.getRightTriggerAxis() > 0.15);
    intakeReverse.onTrue(IntakeCommands.reverseIntake(spindexer));
    intakeReverse.onFalse(IntakeCommands.stopIntake(spindexer));

    Trigger shootTrigger = new Trigger(() -> operator.getRawButton(2));
    shootTrigger.onTrue(ShootingCommands.shoot(spindexer, towerIndexer, shooter));
    shootTrigger.onFalse(ShootingCommands.stopShoot(spindexer, towerIndexer, shooter));
  }

  private void defaultCommands() {
    swerve.setDefaultCommand(
        swerve.driveCommand(
            () -> -xboxController.getLeftY(),
            () -> -xboxController.getLeftX(),
            () -> -xboxController.getRightX()));
  }

  public void setIsBlue() {
    swerve.setIsBlue(DriverStation.getAlliance().map(a -> a == Alliance.Blue).orElse(false));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}