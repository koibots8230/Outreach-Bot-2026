package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TowerIndexerConstants;

public class TowerIndexer extends SubsystemBase{
    @NotLogged private final SparkMax motor;
    @NotLogged private final SparkMaxConfig config;

    @NotLogged private final RelativeEncoder encoder;

    private AngularVelocity setpoint;
    private AngularVelocity velocity;

    private Current current;
    private Voltage appliedVoltage;

    public TowerIndexer() {
        motor = new SparkMax(TowerIndexerConstants.MOTOR_ID, MotorType.kBrushless);

        config = new SparkMaxConfig();

        config.inverted(false);
        config.smartCurrentLimit((int) TowerIndexerConstants.CURRENT_LIMIT.in(Amps));

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder = motor.getEncoder();

        setpoint = RPM.of(0);
        velocity = RPM.of(0);

        current = Amps.of(0);
        appliedVoltage = Volts.of(0);
    }

    @Override
    public void periodic() {
        current = Amps.of(motor.getOutputCurrent());
        velocity = RPM.of(encoder.getVelocity());
        appliedVoltage = Volts.of(motor.getAppliedOutput() * motor.getBusVoltage());
    }

    @Override
    public void simulationPeriodic() {
        velocity = setpoint;
    }

    private void setVelocity(AngularVelocity velocity) {
        setpoint = velocity;
    }

    public Command setVelocityCommand(AngularVelocity velocity) {
        return Commands.runOnce(() -> this.setVelocity(velocity), this);
    }
}