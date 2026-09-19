package frc.robot.commands;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.SpindexerConstants;
import frc.robot.subsystems.Spindexer;

public class intakeCommands {
    public static Command startIntake(Spindexer spindexer) {
        return spindexer.setVelocityCommand(SpindexerConstants.INTAKE_VELOCITY);
    }
}
