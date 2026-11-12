package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

@Logged
public class Intake extends SubsystemBase {

    @NotLogged private final SparkMax topMotor;
    @NotLogged private final SparkMax bottomMotor;
    @NotLogged private final SparkMaxConfig topConfig;
    @NotLogged private final SparkMaxConfig bottomConfig;
    private AngularVelocity setpoint;
    private Current topCurrent, bottomCurrent;
    private Voltage topVoltage, bottomVoltage;
    private AngularVelocity topVelocity, bottomVelocity;
    
    public Intake() {
        topMotor = new SparkMax(IntakeConstants.TOP_MOTOR_ID, MotorType.kBrushless);
        bottomMotor = new SparkMax(IntakeConstants.BOTTOM_MOTOR_ID, MotorType.kBrushless);
        
        topConfig = new SparkMaxConfig();
        topConfig.inverted(true);
        topConfig.smartCurrentLimit((int) IntakeConstants.CURRENT_LIMIT.in(Amps));

        bottomConfig = new SparkMaxConfig();
        bottomConfig.inverted(false);
        bottomConfig.smartCurrentLimit((int) IntakeConstants.CURRENT_LIMIT.in(Amps));

        topMotor.configure(topConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        bottomMotor.configure(bottomConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        setpoint = RPM.of(0);

        topVelocity = RPM.of(0);
        bottomVelocity = RPM.of(0);

        topCurrent = Amps.of(0);
        bottomCurrent = Amps.of(0);

        topVoltage = Volts.of(0);
        bottomVoltage = Volts.of(0);

    }

    @Override
    public void periodic() {
        topCurrent = Amps.of(topMotor.getOutputCurrent());
        topVoltage = Volts.of(topMotor.getAppliedOutput() * topMotor.getBusVoltage());
        topVelocity = RPM.of(topMotor.getEncoder().getVelocity());

        bottomCurrent = Amps.of(bottomMotor.getOutputCurrent());
        bottomVoltage = Volts.of(bottomMotor.getAppliedOutput() * bottomMotor.getBusVoltage());
        bottomVelocity = RPM.of(bottomMotor.getEncoder().getVelocity());
    }
    
}
