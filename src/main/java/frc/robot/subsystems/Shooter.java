package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

@Logged
public class Shooter extends SubsystemBase {

  @NotLogged private final SparkFlex motor;
  @NotLogged private final SparkFlexConfig motorConfig;
  @NotLogged private final SparkClosedLoopController motorController;

  public Shooter() {
    motor = new SparkFlex(ShooterConstants.MOTOR_PORT, MotorType.kBrushless);
    motorConfig = new SparkFlexConfig();

    motorConfig.smartCurrentLimit((int) ShooterConstants.CURRENT_LIMIT.in(Amps));

    motorConfig.inverted(true);

    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motorController = motor.getClosedLoopController();
  }import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

}