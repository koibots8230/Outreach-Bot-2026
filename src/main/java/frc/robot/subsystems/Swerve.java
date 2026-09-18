package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;

@Logged
public class Swerve extends SubsystemBase {

    private Pose2d estimatedPose;
    private Pigeon2 gyro;
    @NotLogged private boolean isBlue;
    private Rotation2d simHeading;
    private Rotation2d gyroHeading;
    private ChassisSpeeds chassisSpeeds;
    private SwerveModuleState[] swerveModuleStates;
    private SwerveModuleState[] realModuleStates;
    private SwerveModulePosition[] swerveModulePositions;
    private final Modules modules;
    private final SwerveDrivePoseEstimator swerveDriveEstimatedPose;

    public Swerve () {
        estimatedPose = new Pose2d();
        simHeading = new Rotation2d();
        gyroHeading = new Rotation2d();
        chassisSpeeds = new ChassisSpeeds();
        modules = new Modules();

        swerveModuleStates = new SwerveModuleState[4];
        realModuleStates = new SwerveModuleState[4];
        swerveModulePositions = new SwerveModulePosition[4];

        gyro = new Pigeon2(SwerveConstants.GYRO_ID);
        gyroHeading = gyro.getRotation2d();

        swerveDriveEstimatedPose =
        new SwerveDrivePoseEstimator(
            SwerveConstants.KINEMATICS, simHeading, getModulePositions(), estimatedPose);
    }

    public class Modules {
        public final SwerveModule frontLeft;
        public final SwerveModule frontRight;
        public SwerveModule backLeft;
        public SwerveModule backRight;

        public Modules() {
            frontLeft = new SwerveModule(SwerveConstants.FRONT_LEFT_DRIVE_ID, SwerveConstants.FRONT_LEFT_TURN_ID);
            frontRight = new SwerveModule(SwerveConstants.FRONT_RIGHT_DRIVE_ID, SwerveConstants.FRONT_RIGHT_TURN_ID);
            backLeft = new SwerveModule(SwerveConstants.BACK_LEFT_DRIVE_ID, SwerveConstants.BACK_LEFT_TURN_ID);
            backRight = new SwerveModule(SwerveConstants.BACK_RIGHT_DRIVE_ID, SwerveConstants.BACK_RIGHT_TURN_ID);
        }
    }

    public void setIsBlue (boolean color) {
        isBlue = color;
    }

    public void zeroGyro(boolean isBlue) {
        new Rotation2d();
    }

    public SwerveModulePosition[] getModulePositions() {
        swerveModulePositions[0] = modules.frontLeft.getModulePosition();
        swerveModulePositions[1] = modules.frontRight.getModulePosition();
        swerveModulePositions[2] = modules.backLeft.getModulePosition();
        swerveModulePositions[3] = modules.backRight.getModulePosition();
        return swerveModulePositions;
    }

    private void driveFieldRelative (LinearVelocity x, LinearVelocity y, AngularVelocity omega) {
        simHeading = new Rotation2d(-omega.baseUnitMagnitude()).div(50).plus(simHeading);
        swerveDriveEstimatedPose.update(simHeading, getModulePositions());
        estimatedPose = swerveDriveEstimatedPose.getEstimatedPosition();
        chassisSpeeds =
        edu.wpi.first.math.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(
            x.in(MetersPerSecond) * SwerveConstants.MAX_LINEAR_VELOCITY.baseUnitMagnitude(),
            y.in(MetersPerSecond) * SwerveConstants.MAX_LINEAR_VELOCITY.baseUnitMagnitude(),
            omega.in(RotationsPerSecond) * SwerveConstants.MAX_ANGULAR_VELOCITY.baseUnitMagnitude(),
            simHeading);

        swerveModuleStates = SwerveConstants.KINEMATICS.toSwerveModuleStates(chassisSpeeds);

        modules.frontLeft.setState(swerveModuleStates[0]);
        modules.frontRight.setState(swerveModuleStates[1]);
        modules.backLeft.setState(swerveModuleStates[2]);
        modules.backRight.setState(swerveModuleStates[3]);

        realModuleStates[0] = modules.frontLeft.getState();
        realModuleStates[1] = modules.frontRight.getState();
        realModuleStates[2] = modules.backLeft.getState();
        realModuleStates[3] = modules.backRight.getState();
    }

    @Override
    public void periodic () {
        modules.frontLeft.periodic();
        modules.frontRight.periodic();
        modules.backLeft.periodic();
        modules.backRight.periodic();

        gyroHeading = gyro.getRotation2d();
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
