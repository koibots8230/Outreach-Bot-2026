package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.SwerveConstants;

@Logged
public class SwerveModule extends SubsystemBase {
    private Angle turnSetpointAngle;
    private LinearVelocity driveSetpointVelocity;
    double position;
    Rotation2d angle;

    @NotLogged private final SparkFlex driveMotor;
    @NotLogged private final SparkFlexConfig driveConfig;
    @NotLogged private final RelativeEncoder driveEncoder;
    @NotLogged private final SparkClosedLoopController driveClosedLoopController;
    
    @NotLogged private final SparkMax turnMotor;
    @NotLogged private final SparkMaxConfig turnConfig;
    @NotLogged private final AbsoluteEncoder turnEncoder;
    @NotLogged private final SparkClosedLoopController turnClosedLoopController;

    @NotLogged SimpleMotorFeedforward turnFeedforward;

    @NotLogged TrapezoidProfile turnProfile;
    private TrapezoidProfile.State turnGoalState;
    private TrapezoidProfile.State turnSetpointState;

    private Angle turnSetpoint;
    private LinearVelocity driveSetpoint;

    private final Rotation2d offset;

    double drivePosition;
    double turnPosition;
    double driveVelocity;
    private AngularVelocity turnVelocity;

    private Current driveCurrent;
    private Current turnCurrent;

    private Voltage driveVoltage;
    private Voltage turnVoltage;

    
    
    public SwerveModule(int driveID, int turnID) {

        offset = switch (driveID) {
            case SwerveConstants.FRONT_LEFT_DRIVE_ID -> SwerveConstants.OFFSETS[0];
            case SwerveConstants.FRONT_RIGHT_DRIVE_ID -> SwerveConstants.OFFSETS[1];
            case SwerveConstants.BACK_LEFT_DRIVE_ID -> SwerveConstants.OFFSETS[2];
            case SwerveConstants.BACK_RIGHT_DRIVE_ID -> SwerveConstants.OFFSETS[3];
            default -> {
                System.out.println("Given Drive ID is not present in SwerveConstants");
                yield new Rotation2d();
            }

        };

        position = 0;
        angle = new Rotation2d();
        
        driveMotor = new SparkFlex(driveID, MotorType.kBrushless);
        turnMotor = new SparkMax(turnID, MotorType.kBrushless);

        driveConfig = new SparkFlexConfig();
        driveConfig.closedLoop.pid(SwerveConstants.DRIVE_PID.kp,
            SwerveConstants.DRIVE_PID.ki,
            SwerveConstants.DRIVE_PID.kd);
        driveConfig.idleMode(IdleMode.kBrake);
        driveConfig.smartCurrentLimit((int) SwerveConstants.DRIVE_CURRENT_LIMIT.in(Amps));
        driveConfig.encoder.positionConversionFactor(SwerveConstants.DRIVE_CONVERSION_FACTOR);
        driveConfig.encoder.velocityConversionFactor(SwerveConstants.DRIVE_CONVERSION_FACTOR / 60.0);
        driveClosedLoopController = driveMotor.getClosedLoopController();

        turnConfig = new SparkMaxConfig();
        turnConfig.idleMode(IdleMode.kBrake);
        turnSetpointState = new TrapezoidProfile.State(0,0);
        turnConfig
            .closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pid(SwerveConstants.DRIVE_PID.kp, SwerveConstants.DRIVE_PID.ki, SwerveConstants.DRIVE_PID.kd)
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(-Math.PI, Math.PI);
        turnConfig.smartCurrentLimit((int) SwerveConstants.TURN_CURRENT_LIMIT.in(Amps));
        turnConfig.absoluteEncoder.positionConversionFactor(SwerveConstants.TURN_CONVERSION_FACTOR);
        turnConfig.absoluteEncoder.velocityConversionFactor(SwerveConstants.TURN_CONVERSION_FACTOR / 60);
        turnConfig.absoluteEncoder.inverted(true);
        turnClosedLoopController = turnMotor.getClosedLoopController();

        turnProfile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                SwerveConstants.MAX_TURN_ACCELERATION, SwerveConstants.MAX_TURN_VELOCITY));

        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        turnMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        driveEncoder = driveMotor.getEncoder();
        turnEncoder = turnMotor.getAbsoluteEncoder();

        turnFeedforward =
            new SimpleMotorFeedforward(SwerveConstants.TURN_FEEDFORWARD.ks, SwerveConstants.TURN_FEEDFORWARD.kv);
        
        driveSetpoint = LinearVelocity.ofBaseUnits(0.0, Units.MetersPerSecond);
        turnSetpoint = Radians.of(0);
        drivePosition = driveEncoder.getPosition();
        turnPosition = turnEncoder.getPosition();
        driveVelocity = driveEncoder.getVelocity();
        turnVelocity = AngularVelocity.ofBaseUnits(turnEncoder.getVelocity(), Units.RadiansPerSecond);
        driveVoltage =
            Voltage.ofBaseUnits(driveMotor.getAppliedOutput() * driveMotor.getBusVoltage(), Volts);
        turnVoltage =
            Voltage.ofBaseUnits(turnMotor.getAppliedOutput() * turnMotor.getBusVoltage(), Volts);
        driveCurrent = Current.ofBaseUnits(driveMotor.getOutputCurrent(), Amps);
        turnCurrent = Current.ofBaseUnits(turnMotor.getOutputCurrent(), Amps);

    }
    
    public void setState(SwerveModuleState state) {
        driveSetpointVelocity = MetersPerSecond.of(state.speedMetersPerSecond);
        turnSetpointAngle = Radians.of(state.angle.getRadians());
        position = (driveSetpointVelocity.baseUnitMagnitude());
        driveClosedLoopController.setSetpoint(state.speedMetersPerSecond, SparkBase.ControlType.kVelocity);
        turnClosedLoopController.setSetpoint(state.angle.getRadians(), ControlType.kPosition);
    }

    @Override
    public void periodic() {
        drivePosition = driveEncoder.getPosition();
        driveVelocity = driveEncoder.getVelocity();
        driveVoltage =
            Voltage.ofBaseUnits(driveMotor.getBusVoltage(), Volts);
        driveCurrent = Current.ofBaseUnits(driveMotor.getOutputCurrent(), Amps);

        turnPosition = turnEncoder.getPosition() - offset.getRadians();
        turnVelocity = AngularVelocity.ofBaseUnits(turnEncoder.getVelocity(), Units.RadiansPerSecond);
        turnVoltage =
            Voltage.ofBaseUnits(turnMotor.getBusVoltage(), Volts);
        turnCurrent = Current.ofBaseUnits(driveMotor.getOutputCurrent(), Amps);

        turnGoalState =
        new TrapezoidProfile.State(
            MathUtil.angleModulus(turnSetpoint.in(Radians) + offset.getRadians()), 0);
        turnSetpointState =
         turnProfile.calculate(
             RobotConstants.CLOCK_SPEED.in(Seconds), turnSetpointState, turnGoalState);

    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(driveSetpointVelocity, new Rotation2d(turnSetpointAngle));
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(drivePosition, new Rotation2d(turnPosition));
    }

}

