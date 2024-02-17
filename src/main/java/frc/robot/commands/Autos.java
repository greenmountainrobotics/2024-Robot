package frc.robot.commands;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.ShooterSimple;
import frc.robot.util.Alliance;
import frc.robot.util.FieldPoseUtils;
import java.util.Set;
import org.littletonrobotics.junction.Logger;

public class Autos {
  enum Trajectory {
    AmpToMiddle("Amp to Middle (Week 0)"),
    AmpToSource("Amp to Source"),
    FarSideToAmp("Far side to Amp"),
    FarSideToSource("Far side to Source");

    private final String fileName;

    Trajectory(String fileName) {
      this.fileName = fileName;
    }
  }

  private static Command followPath(Drive drive, Trajectory trajectoryFile) {
    ChoreoTrajectory trajectory = Choreo.getTrajectory(trajectoryFile.fileName);

    return new SequentialCommandGroup(
        new DriveToPose(drive, FieldPoseUtils.flipPoseIfRed(trajectory.getInitialPose())),
        new InstantCommand(
            () ->
                Logger.recordOutput(
                    "Auto/TargetPose", FieldPoseUtils.flipPoseIfRed(trajectory.getFinalPose()))),
        Choreo.choreoSwerveCommand(
            trajectory,
            drive::getPose,
            new PIDController(Constants.DrivePIDConstants.KpX, 0, 0),
            new PIDController(Constants.DrivePIDConstants.KpY, 0, 0),
            new PIDController(Constants.DrivePIDConstants.KpTheta, 0, 0),
            drive::runVelocity,
            Alliance::isRed));
  }

  private static double WAIT_DURATION = 9;

  public static Command CloseSideToAmp(Drive drive, ShooterSimple shooter) {
    return new DeferredCommand(
        () ->
            new SequentialCommandGroup(
                new DriveToPose(drive, FieldPoseUtils.alignedWithAmpPose()), ShootInAmp(shooter)),
        Set.of(drive, shooter));
  }

  public static Command FarSideToAmp(Drive drive, ShooterSimple shooter) {
    return new DeferredCommand(
        () ->
            new SequentialCommandGroup(
                followPath(drive, Trajectory.FarSideToAmp),
                new DriveToPose(drive, FieldPoseUtils.alignedWithAmpPose()),
                ShootInAmp(shooter)),
        Set.of(drive, shooter));
  }

  public static Command CloseSideToAmpToSource(Drive drive, ShooterSimple shooter) {
    return new DeferredCommand(
        () ->
            new SequentialCommandGroup(
                new DriveToPose(drive, FieldPoseUtils.alignedWithAmpPose()),
                ShootInAmp(shooter),
                followPath(drive, Trajectory.AmpToSource),
                new DriveToPose(drive, FieldPoseUtils.alignedWithSourcePose())),
        Set.of(drive, shooter));
  }

  public static Command FarSideToAmpToSource(Drive drive, ShooterSimple shooter) {
    return new DeferredCommand(
        () ->
            new SequentialCommandGroup(
                followPath(drive, Trajectory.FarSideToAmp),
                new DriveToPose(drive, FieldPoseUtils.alignedWithAmpPose()),
                ShootInAmp(shooter),
                followPath(drive, Trajectory.AmpToSource),
                new DriveToPose(drive, FieldPoseUtils.alignedWithSourcePose())),
        Set.of(drive, shooter));
  }

  public static Command CloseSideToAmpToMiddle(Drive drive, ShooterSimple shooter) {
    return new DeferredCommand(
        () ->
            new SequentialCommandGroup(
                new WaitCommand(WAIT_DURATION),
                new DriveToPose(drive, FieldPoseUtils.alignedWithAmpPose()),
                ShootInAmp(shooter),
                followPath(drive, Trajectory.AmpToMiddle)),
        Set.of(drive, shooter));
  }

  public static Command FarSideToAmpToMiddle(Drive drive, ShooterSimple shooter) {
    return new DeferredCommand(
        () ->
            new SequentialCommandGroup(
                followPath(drive, Trajectory.FarSideToAmp),
                new DriveToPose(drive, FieldPoseUtils.alignedWithAmpPose()),
                ShootInAmp(shooter),
                followPath(drive, Trajectory.AmpToMiddle)),
        Set.of(drive, shooter));
  }

  public static Command FarSideToSource(Drive drive) {
    return new DeferredCommand(
        () ->
            new SequentialCommandGroup(
                followPath(drive, Trajectory.FarSideToSource),
                new DriveToPose(drive, FieldPoseUtils.alignedWithSourcePose())),
        Set.of(drive));
  }

  public static Command ShootInAmp(ShooterSimple shooter) {
    return new RunCommand(() -> shooter.setFlywheels(-0.3, 0.3), shooter)
        .withTimeout(0.5)
        .andThen(() -> shooter.setFlywheels(0, 0));
  }
}
