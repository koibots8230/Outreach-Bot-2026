// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import frc.lib.util.FeedforwardGains;
import frc.lib.util.PIDGains;

public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public class ShooterConstants {
    public static final AngularVelocity HIGH_GOAL_SPEED = RPM.of(2100);
    public static final AngularVelocity LOW_GOAL_SPEED = RPM.of(645);

    public static final AngularVelocity REVERSE_SPEED = RPM.of(-500);

    public static final PIDGains PID = new PIDGains.Builder().kp(0.0016).build();
    public static final FeedforwardGains FEEDFORWARD =
        new FeedforwardGains.Builder().kv(0.000185).build();

    public static final Current CURRENT_LIMIT = Amps.of(80);

    public static final int MOTOR_PORT = 31;
  }
}
