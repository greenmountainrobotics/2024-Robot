package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

// TODO: tune constants
public class DriveToPose extends Command {
  private final Drive drive;
  private final Pose2d targetPose;

  private final ProfiledPIDController translationController;
  private final ProfiledPIDController thetaController;

  private static final double driveTolerance = 0.02;
  private static final double thetaTolerance = 0.02;

  public DriveToPose(Drive drive, Pose2d targetPose) {
    this.drive = drive;
    this.targetPose = targetPose;
    addRequirements(drive);

    translationController =
        new ProfiledPIDController(
            Constants.DrivePIDConstants.KpTranslation,
            0,
            0,
            new TrapezoidProfile.Constraints(5, 5));
    translationController.setTolerance(driveTolerance);

    thetaController =
        new ProfiledPIDController(
            Constants.DrivePIDConstants.KpTheta, 0, 0, new TrapezoidProfile.Constraints(5, 5));
    thetaController.setTolerance(thetaTolerance);
  }

  @Override
  public void initialize() {
    Logger.recordOutput("DriveToPose/TargetPose", targetPose);
    var currentPose = drive.getPose();
    translationController.reset(
        currentPose.getTranslation().getDistance(targetPose.getTranslation()),
        -new Translation2d(drive.getFieldVelocity().dx, drive.getFieldVelocity().dy)
            .rotateBy(
                targetPose.getTranslation().minus(drive.getPose().getTranslation()).getAngle())
            .getX());
    thetaController.reset(currentPose.getRotation().getRadians(), drive.getYawVelocity());
  }

  @Override
  public void execute() {
    var currentPose = drive.getPose();

    double currentDistance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
    double translationVelocityScalar = translationController.calculate(currentDistance, 0.0);

    double thetaVelocity =
        thetaController.calculate(
            currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

    var driveVelocity =
        new Pose2d(
                new Translation2d(),
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
            .transformBy(new Transform2d(translationVelocityScalar, 0.0, new Rotation2d()))
            .getTranslation();

    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            driveVelocity.getX(), driveVelocity.getY(), thetaVelocity, currentPose.getRotation()));
  }

  @Override
  public boolean isFinished() {
    var currentPose = drive.getPose();
    return (Math.abs(currentPose.getTranslation().getDistance(targetPose.getTranslation()))
            < driveTolerance
        && Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians())
            < thetaTolerance);
  }
}
