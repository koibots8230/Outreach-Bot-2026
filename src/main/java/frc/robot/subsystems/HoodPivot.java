package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodPivotConstants;
import frc.robot.Constants.RobotConstants;



import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class HoodPivot extends SubsystemBase {
    private final SparkMax motor;
    private final SparkMaxConfig config;

    public HoodPivot () { 
        motor = new SparkMax(HoodPivotConstants.MOTOR_ID, MotorType.kBrushless);
        config = new SparkMaxConfig();
    }

}