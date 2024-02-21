package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.TunableConstants;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

// TODO: tune constants
public class DriveToPose extends Command {
  private final Drive drive;
  private final Supplier<Pose2d> targetPoseSupplier;

  private final ProfiledPIDController translationController;
  private final ProfiledPIDController thetaController;

  public DriveToPose(Drive drive, Supplier<Pose2d> targetPoseSupplier) {
    this.drive = drive;
    this.targetPoseSupplier = targetPoseSupplier;
    addRequirements(drive);

    translationController =
        new ProfiledPIDController(
            TunableConstants.KpTranslation, 0, 0, new TrapezoidProfile.Constraints(5, 5));
    translationController.setTolerance(DriveConstants.DriveTolerance);

    thetaController =
        new ProfiledPIDController(
            TunableConstants.KpTheta, 0, 0, new TrapezoidProfile.Constraints(5, 5));
    thetaController.setTolerance(DriveConstants.ThetaToleranceRad);
  }

  @Override
  public void initialize() {
    var targetPose = targetPoseSupplier.get();
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
    var targetPose = targetPoseSupplier.get();
    Logger.recordOutput("Auto/TargetPose", targetPose);
    Logger.recordOutput("Auto/Trajectory", drive.getPose(), targetPose);

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
  public void end(boolean interrupted) {
    drive.stop();
  }

  @Override
  public boolean isFinished() {
    var currentPose = drive.getPose();
    var targetPose = targetPoseSupplier.get();
    return (Math.abs(currentPose.getTranslation().getDistance(targetPose.getTranslation()))
            < DriveConstants.DriveTolerance
        && Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians())
            < DriveConstants.ThetaToleranceRad);
  }
}
