package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

@Logged
public class Intake extends SubsystemBase {

    @NotLogged private final SparkMax topMotor;
    @NotLogged private final SparkMax bottomMotor;
    @NotLogged private final SparkMaxConfig config;
    @NotLogged private final SparkClosedLoopController topClosedLoopController;
    @NotLogged private final SparkClosedLoopController bottomClosedLoopController;
    private AngularVelocity setpoint;
    private Current current;
    private Voltage voltage;
    private AngularVelocity velocity;
    
    public Intake() {
        topMotor = new SparkMax(IntakeConstants.TOP_MOTOR_ID, null);
        bottomMotor = new SparkMax(IntakeConstants.BOTTOM_MOTOR_ID, null);
        
        config = new SparkMaxConfig();
        config.inverted(true);
        config.smartCurrentLimit((int) IntakeConstants.CURRENT_LIMIT.in(Amps));

        topMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        bottomMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        topClosedLoopController = topMotor.getClosedLoopController();
        bottomClosedLoopController = bottomMotor.getClosedLoopController();

        setpoint = RPM.of(0);
        velocity = RPM.of(0);
        current = Amps.of(0);
        voltage = Volts.of(0);
    }
    
}
