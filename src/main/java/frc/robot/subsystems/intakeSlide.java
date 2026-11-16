package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Encoder;
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
    
    

    public intakeSlide(){
        slideMotor = new SparkMax(intakeSlideConstants.MOTOR_ID, SparkMax.MotorType.kBrushless);
        slideEncoder = slideMotor.getEncoder();
        closedLoopController = slideMotor.getClosedLoopController();
        limitSwitch = slideMotor.getForwardLimitSwitch();

        slideMotorConfig = new SparkMaxConfig();
        slideMotorConfig.idleMode(IdleMode.kBrake);

        slideMotorConfig.limitSwitch
            .forwardLimitSwitchType(Type.kNormallyOpen);

        slideMotorConfig.closedLoop
            .kP(intakeSlideConstants.PID_GAINS.kP);
            .kI(intakeSlideConstants.PID_GAINS.kI);
            .kD(intakeSlideConstants.PID_GAINS.kD);

        slideMotor.configure(slideMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        

        tProfile = new TrapezoidProfile(intakeSlideConstants.TRAPEZOID_PROFILE_CONSTANTS);
        feedforward = new ElevatorFeedforward(intakeSlideConstants.FEEDFORWARD_GAINS);

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
