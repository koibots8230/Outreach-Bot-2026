package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SpindexerConstants;
import frc.robot.subsystems.Spindexer;

public class IntakeCommands {
  public static Command startIntake(Spindexer spindexer) {
    return spindexer.setVelocityCommand(SpindexerConstants.INTAKE_VELOCITY);
  }
}
