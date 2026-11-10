package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class Intake extends SubsystemBase {

    private final SparkMax topMotor;
    private final SparkMax bottomMotor;
    private final SparkMaxConfig motorConfig;
    double setpoint;
    double current;
    double voltage;
    double velocity;
    
    public Intake() {
        topMotor = new SparkMax(11, null);
        bottomMotor = new SparkMax(12, null);
        motorConfig = new SparkMaxConfig();
    }
    
}
