package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.TurretConstants;

@Logged
public class Turret extends SubsystemBase {

  @NotLogged private final SparkMax motor;
  @NotLogged private final SparkMaxConfig config;

  @NotLogged private final AbsoluteEncoder encoder;

  @NotLogged private final SparkClosedLoopController controller;

  @NotLogged private final SimpleMotorFeedforward feedforward;

  @NotLogged private final TrapezoidProfile profile;
  @NotLogged private TrapezoidProfile.State goalState;
  @NotLogged private TrapezoidProfile.State motorSetpoint;

  private Angle setpoint;
  private Angle position;

  private AngularVelocity velocity;

  private Current current;
  private Voltage appliedVoltage;

  public Turret() {
    motor = new SparkMax(TurretConstants.MOTOR_ID, MotorType.kBrushless);

    config = new SparkMaxConfig();

    config.inverted(false);
    config.smartCurrentLimit((int) TurretConstants.CURRENT_LIMIT.in(Amps));

    config.voltageCompensation(RobotConstants.NOMINAL_VOLTAGE.in(Volts));

    config.absoluteEncoder.positionConversionFactor(TurretConstants.CONVERSION_FACTOR);

    config.absoluteEncoder.velocityConversionFactor(TurretConstants.CONVERSION_FACTOR / 60.0);

    config.closedLoop.pid(
        TurretConstants.PID_GAINS.kp, TurretConstants.PID_GAINS.ki, TurretConstants.PID_GAINS.kd);

    config.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    encoder = motor.getAbsoluteEncoder();

    controller = motor.getClosedLoopController();

    feedforward =
        new SimpleMotorFeedforward(
            TurretConstants.FEEDFORWARD_GAINS.ks, TurretConstants.FEEDFORWARD_GAINS.kv);

    profile =
        new TrapezoidProfile(
            new Constraints(
                TurretConstants.MAX_VELOCITY.in(RadiansPerSecond),
                TurretConstants.MAX_ACCELERATION.in(RadiansPerSecondPerSecond)));

    goalState = new State(0, 0);

    motorSetpoint = new State(0, 0);

    setpoint = Radians.of(0);
    position = Radians.of(0);

    velocity = RadiansPerSecond.of(0);

    current = Amps.of(0);
    appliedVoltage = Volts.of(0);
  }

  @Override
  public void periodic() {
    motorSetpoint =
        profile.calculate(RobotConstants.CLOCK_SPEED.in(Seconds), motorSetpoint, goalState);

    controller.setSetpoint(
        motorSetpoint.position,
        ControlType.kPosition,
        ClosedLoopSlot.kSlot0,
        feedforward.calculate(motorSetpoint.velocity));

    position = Radians.of(encoder.getPosition());
    velocity = RadiansPerSecond.of(encoder.getVelocity());

    current = Amps.of(motor.getOutputCurrent());
    appliedVoltage = Volts.of(motor.getAppliedOutput() * motor.getBusVoltage());
  }

  @Override
  public void simulationPeriodic() {
    position = Radians.of(motorSetpoint.position);
    velocity = RadiansPerSecond.of(motorSetpoint.velocity);
  }

  private void setPosition(Angle angle) {
    setpoint = angle;
    goalState = new State(angle.in(Radians), 0);
  }

  public Command setPositionCommand(Angle angle) {
    return Commands.runOnce(() -> this.setPosition(angle), this);
  }
}
