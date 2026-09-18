package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.*;
import frc.robot.subsystems.*;

@Logged
public class RobotContainer {
  @NotLogged private final XboxController controller;
  private final Spindexer spindexer;

  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  public RobotContainer() {
    controller = new XboxController(0);
    spindexer = new Spindexer();

    configureBindings();
  }

  private void configureBindings() {
    Trigger intakeButton = new Trigger(() -> controller.getLeftTriggerAxis() > 0.15);
    intakeButton.onTrue(spindexer.setVelocityCommand(SpindexerConstants.INTAKE_VELOCITY));
    intakeButton.onFalse(spindexer.setVelocityCommand(RPM.of(0)));

    Trigger intakeReverse = new Trigger(() -> controller.getRightTriggerAxis() > 0.15);
    intakeReverse.onTrue(spindexer.setVelocityCommand(SpindexerConstants.REVERSE_VELOCITY));
    intakeReverse.onFalse(spindexer.setVelocityCommand(RPM.of(0)));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
