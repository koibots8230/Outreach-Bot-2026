package frc.robot.commands;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.SpindexerConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Spindexer;

public class IntakeCommands {
  public static Command startIntake(Spindexer spindexer, Intake intake) {
    return Commands.sequence(
        spindexer.setVelocityCommand(SpindexerConstants.INTAKE_VELOCITY),
        intake.setVelocityCommand(IntakeConstants.INTAKE_VELOCITY));
  }

  public static Command reverseIntake(Spindexer spindexer, Intake intake) {
    return Commands.sequence(
        spindexer.setVelocityCommand(SpindexerConstants.REVERSE_VELOCITY),
        intake.setVelocityCommand(IntakeConstants.REVERSE_VELOCITY));
  }

  public static Command stopIntake(Spindexer spindexer, Intake intake) {
    return Commands.sequence(
      spindexer.setVelocityCommand(RPM.of(0)),
      intake.setVelocityCommand(RPM.of(0)));
  }
}
