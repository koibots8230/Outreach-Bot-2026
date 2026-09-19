package frc.robot;

import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.HoodPivotConstants;
import frc.robot.subsystems.HoodPivot;

public class RobotContainer {
  @NotLogged private final XboxController controller;
  @NotLogged private final GenericHID operator;

  private final HoodPivot hoodPivot;


  public RobotContainer() {
    hoodPivot = new HoodPivot();

    controller = new XboxController(0);
    operator = new GenericHID(1);

    configureBindings();
  }

  private void configureBindings() {
    Trigger pivotUp = new Trigger(() -> operator.getRawButton(8));
    pivotUp.onTrue(hoodPivot.setPositionCommand(HoodPivotConstants.UP_POSITION));

    Trigger pivotDown = new Trigger(() -> operator.getRawButton(7));
    pivotDown.onTrue(hoodPivot.setPositionCommand(HoodPivotConstants.DOWN_POSITION));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
