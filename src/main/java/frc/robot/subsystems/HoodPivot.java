package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodPivotConstants;
import frc.robot.Constants.RobotConstants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;

@Logged
public class HoodPivot extends SubsystemBase {
    private final SparkMax motor;
    private final SparkMaxConfig config;
    private final TrapezoidProfile profile;
    private TrapezoidProfile.State goal;
    private TrapezoidProfile.State motorSetpoint;
    double position;
    double setpoint;
    double current;
    double voltage;


    public HoodPivot () { 
        motor = new SparkMax(HoodPivotConstants.MOTOR_ID, MotorType.kBrushless);

        config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        config.inverted(false);
        config.smartCurrentLimit((int) HoodPivotConstants.CURRENT_LIMIT.in(Amps));
        
        config.absoluteEncoder.positionConversionFactor(HoodPivotConstants.CONVERSION_FACTOR);
        config.absoluteEncoder.inverted(false);
        
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        profile = new TrapezoidProfile(
            new Constraints(
                HoodPivotConstants.MAX_VELOCITY.in(RadiansPerSecond),
                HoodPivotConstants.MAX_ACCELERATION.in(RadiansPerSecondPerSecond)));
        goal = new State(motor.getAbsoluteEncoder().getPosition(), 0);
        motorSetpoint = new State(motor.getAbsoluteEncoder().getPosition(),0);
    }

    @Override
    public void periodic() {
        motorSetpoint = profile.calculate(RobotConstants.CLOCK_SPEED.in(Seconds), motorSetpoint, goal);
        position = motor.getAbsoluteEncoder().getPosition();
        current = motor.getOutputCurrent();
        voltage = motor.getAppliedOutput() * motor.getBusVoltage();
    }

        public boolean atSetpoint() {
        return (position >= (setpoint - HoodPivotConstants.TOLERANCE.getRadians())
            && position <= (setpoint + HoodPivotConstants.TOLERANCE.getRadians()));
    }

    @Override
    public void simulationPeriodic() {
        position = motorSetpoint.position;
    }

    private void setPosition(Rotation2d angle) {
        goal = new State(angle.getRadians(), 0);
        setpoint = angle.getRadians();
    }

    public Command setPositionCommand(Rotation2d angle) {
        return Commands.runOnce(() -> this.setPosition(angle), this);
    }

}