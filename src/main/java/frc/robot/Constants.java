// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import frc.lib.util.FeedforwardGains;
import frc.lib.util.PIDGains;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static class HoodPivotConstants {
    public static final Rotation2d UP_POSITION = Rotation2d.fromDegrees(240); //might change these angles
    public static final Rotation2d MID_POSITION = Rotation2d.fromDegrees(120); 
    public static final Rotation2d DOWN_POSITION = Rotation2d.fromDegrees(0.01);
    public static final Rotation2d TOLERANCE = Rotation2d.fromRadians(0.15);
    public static final PIDGains PID = new PIDGains.Builder().kp(0.1/* placeholder */).build(); 
    public static final FeedforwardGains FEEDFORWARD =
        new FeedforwardGains.Builder().kv(0).kg(0).build();
    public static final double CONVERSION_FACTOR = Math.PI * 2; //might change (or not, idk)

    public static final AngularVelocity MAX_VELOCITY = DegreesPerSecond.of(360);
    public static final AngularAcceleration MAX_ACCELERATION = DegreesPerSecondPerSecond.of(180);
    public static final Current CURRENT_LIMIT = Amps.of(60);

    public static final int MOTOR_ID = 32;
  }

  public final class RobotConstants {
    public static final Time CLOCK_SPEED = Milliseconds.of(20);
  }
}
