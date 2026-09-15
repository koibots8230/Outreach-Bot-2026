package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.function.DoubleSupplier;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class Swerve extends SubsystemBase {

    private Pose2d estimatedPose;
    private boolean isBlue;

    public Swerve () {
        estimatedPose = new Pose2d();
    }

    public void setIsBlue (boolean color) {
        isBlue = color;
    }

    private void driveFieldRelative (LinearVelocity x, LinearVelocity y, AngularVelocity omega) {
        estimatedPose = 
             new Pose2d(
                ((x.baseUnitMagnitude() / 50 * 4) * (isBlue ? -1 : 1) + (estimatedPose.getX())),
                ((y.baseUnitMagnitude() / 50 * 4) * (isBlue ? -1 : 1) + (estimatedPose.getY())),
                new Rotation2d((-omega.baseUnitMagnitude() / 50) + (estimatedPose.getRotation().getRadians()))
            );
    }

    public Command driveCommand (DoubleSupplier x, DoubleSupplier y, DoubleSupplier omega) {
        return Commands.run(
            () ->
            driveFieldRelative(
                MetersPerSecond.of(x.getAsDouble()), 
                MetersPerSecond.of(y.getAsDouble()),
                RadiansPerSecond.of(omega.getAsDouble())),
            this);
    }

}
