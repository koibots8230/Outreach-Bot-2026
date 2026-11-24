package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.intakeSlideConstants;

public class intakeSlide extends SubsystemBase {
  private SparkMax slideMotor;
  private SparkMaxConfig slideMotorConfig;
  private RelativeEncoder slideEncoder;
  private SparkClosedLoopController closedLoopController;
  private final SparkLimitSwitch limitSwitch;
  private final TrapezoidProfile tProfile;
  private TrapezoidProfile.State goalState;
  private TrapezoidProfile.State motorSetpoint;
  private final ElevatorFeedforward feedforward;

  private Current current;
  private Voltage voltage;
  private Angle position;

  public intakeSlide() {
    slideMotor = new SparkMax(intakeSlideConstants.MOTOR_ID, SparkMax.MotorType.kBrushless);
    slideEncoder = slideMotor.getEncoder();
    closedLoopController = slideMotor.getClosedLoopController();
    limitSwitch = slideMotor.getForwardLimitSwitch();

    slideMotorConfig = new SparkMaxConfig();
    slideMotorConfig.idleMode(IdleMode.kBrake);

    slideMotorConfig.limitSwitch.forwardLimitSwitchType(Type.kNormallyOpen);

    slideMotorConfig
        .closedLoop
        .p(intakeSlideConstants.PID_GAINS.kp)
        .i(intakeSlideConstants.PID_GAINS.ki)
        .d(intakeSlideConstants.PID_GAINS.kd);

    slideMotor.configure(
        slideMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    feedforward =
        new ElevatorFeedforward(
            intakeSlideConstants.FEEDFORWARD_GAINS.ks,
            intakeSlideConstants.FEEDFORWARD_GAINS.kv,
            intakeSlideConstants.FEEDFORWARD_GAINS.ka);

    tProfile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                intakeSlideConstants.MAX_VELOCITY, intakeSlideConstants.MAX_ACCELERATION));
  }

  private void setIntakePosition(double targetPosition) {
    closedLoopController.setSetpoint(targetPosition, ControlType.kPosition);
  }

  public Command extendIntake() {
    return Commands.runOnce(() -> setIntakePosition(intakeSlideConstants.EXTENDED_POSITION), this);
  }

  public Command retractIntake() {
    return Commands.runOnce(() -> setIntakePosition(intakeSlideConstants.RETRACTED_POSITION), this);
  }

  public Command setZero() {
    return Commands.runOnce(() -> slideEncoder.setPosition(0.0), this);
  }

  public Command stopSlide() {
    return Commands.runOnce(slideMotor::stopMotor, this);
  }
}
