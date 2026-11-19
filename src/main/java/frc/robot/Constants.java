// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.util.FeedforwardGains;
import frc.lib.util.PIDGains;

public final class Constants {
  public static class RobotConstants {
    public static final int CONTROLLER_PORT = 0;

    public static final double DRIVETRAIN_LENGTH = Units.inchesToMeters(27);
    public static final double DRIVETRAIN_WIDTH = Units.inchesToMeters(27);

    public static final Time CLOCK_SPEED = Seconds.of(0.02);

    public static final Voltage NOMINAL_VOLTAGE = Volts.of(12);
  }

  public static class SwerveConstants {

    public static final LinearVelocity MAX_LINEAR_VELOCITY = MetersPerSecond.of(4);
    public static final AngularVelocity MAX_ANGULAR_VELOCITY = RadiansPerSecond.of(2 * Math.PI);

    public static final SwerveDriveKinematics KINEMATICS =
        new SwerveDriveKinematics(
            new Translation2d(
                RobotConstants.DRIVETRAIN_LENGTH / 2, RobotConstants.DRIVETRAIN_WIDTH / 2),
            new Translation2d(
                RobotConstants.DRIVETRAIN_LENGTH / 2, -RobotConstants.DRIVETRAIN_WIDTH / 2),
            new Translation2d(
                -RobotConstants.DRIVETRAIN_LENGTH / 2, RobotConstants.DRIVETRAIN_WIDTH / 2),
            new Translation2d(
                -RobotConstants.DRIVETRAIN_LENGTH / 2, -RobotConstants.DRIVETRAIN_WIDTH / 2));

    public static final double MAX_TURN_ACCELERATION = 30 * Math.PI;
    public static final double MAX_TURN_VELOCITY = 20 * Math.PI;

    public static final PIDGains DRIVE_PID = new PIDGains.Builder().kp(0).ki(0).kd(0).build();
    public static final FeedforwardGains DRIVE_FEEDFORWARD =
        new FeedforwardGains.Builder().ks(0).kv(0.05).build();

    public static final PIDGains TURN_PID = new PIDGains.Builder().kp(0).ki(0).kd(0).build();
    public static final FeedforwardGains TURN_FEEDFORWARD =
        new FeedforwardGains.Builder().ks(0).kv(0.05).build();

    public static final Current DRIVE_CURRENT_LIMIT = Current.ofBaseUnits(80, Amps);
    public static final Current TURN_CURRENT_LIMIT = Current.ofBaseUnits(60, Amps);

    public static final double SWERVE_GEARING = 5.50;

    public static final double DRIVE_CONVERSION_FACTOR =
        (Units.inchesToMeters(1.5) * 2 * Math.PI / SWERVE_GEARING);
    public static final double TURN_CONVERSION_FACTOR = 2 * Math.PI;

    public static final Rotation2d[] OFFSETS = {
      Rotation2d.fromRadians(0), // Front Left
      Rotation2d.fromRadians(0), // Front Right
      Rotation2d.fromRadians(0), // Back Left
      Rotation2d.fromRadians(0) // Back Right
    };

    public static final int FRONT_LEFT_DRIVE_ID = 1;
    public static final int FRONT_LEFT_TURN_ID = 2;

    public static final int FRONT_RIGHT_DRIVE_ID = 3;
    public static final int FRONT_RIGHT_TURN_ID = 4;

    public static final int BACK_LEFT_DRIVE_ID = 5;
    public static final int BACK_LEFT_TURN_ID = 6;

    public static final int BACK_RIGHT_DRIVE_ID = 7;
    public static final int BACK_RIGHT_TURN_ID = 8;

    public static final int GYRO_ID = 9;
  }

  public static class SpindexerConstants {
    public static final AngularVelocity INTAKE_VELOCITY = RPM.of(120);
    public static final AngularVelocity SHOOTING_VELOCITY = RPM.of(240);
    public static final AngularVelocity REVERSE_VELOCITY = RPM.of(-120);
    public static final Current CURRENT_LIMIT = Amps.of(60);

    public static final int MOTOR_ID = 21;
  }

  public static class TowerIndexerConstants {
    public static final AngularVelocity VELOCITY = RPM.of(240);
    public static final Current CURRENT_LIMIT = Amps.of(60);

    public static final int MOTOR_ID = 22;
  }

  public class ShooterConstants {
    public static final AngularVelocity SHOOT_SPEED = RPM.of(1500);

    public static final AngularVelocity REVERSE_SPEED = RPM.of(-500);

    public static final PIDGains PID = new PIDGains.Builder().kp(0.0016).build();
    public static final FeedforwardGains FEEDFORWARD =
        new FeedforwardGains.Builder().kv(0.000185).build();

    public static final Current CURRENT_LIMIT = Amps.of(80);

    public static final int MOTOR_PORT = 31;
  }

  public static class TurretConstants {
    public static final AngularVelocity MAX_VELOCITY = RadiansPerSecond.of(Math.PI * 20);
    public static final AngularAcceleration MAX_ACCELERATION =
        RadiansPerSecondPerSecond.of(Math.PI * 18);

    public static final Angle MAX_ANGLE = Degrees.of(45); // safe placeholder
    // needs to be actually measured on the robot w/ limit switch

    public static final PIDGains PID_GAINS = new PIDGains.Builder().kp(0.0).build();
    public static final FeedforwardGains FEEDFORWARD_GAINS =
        new FeedforwardGains.Builder().kv(0.0).build();

    // 1 encoder rot = 95/12 turret rot, conversion factor converts encoder rot -> turret rad
    // need to verify on robot probably
    // public static final double CONVERSION_FACTOR = 2 * Math.PI / (95.0 / 12.0);
    public static final double MOTOR_TO_TURRET_RATIO = (38/95);
    public static final double CONVERSION_FACTOR = 2 * Math.PI / (MOTOR_TO_TURRET_RATIO);
    public static final Current CURRENT_LIMIT = Amps.of(60);

    public static final int MOTOR_ID = 33;
    public static final int LEFT_LIMIT_PORT = 8; // idk what this number actually is
    public static final int RIGHT_LIMIT_PORT = 9; // or this
  }
}
