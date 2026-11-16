package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class intakeSlide extends SubsystemBase {
    private SparkFlex slideMotor;
    private SparkFlexConfig slideMotorConfig;

    private Current current;
    private Voltage voltage;
    private Angle position;
    
    

    public intakeSlide(){

    }

    private void setIntakePosition() {
        // Code to set the intake position
    }

    public Command extendIntake() {
        return Commands.runOnce(() -> {
            // Code to extend the intake
        });
    }

    public Command retractIntake() {
        return Commands.runOnce(() -> {
            // Code to retract the intake
        });
    }
    
}
