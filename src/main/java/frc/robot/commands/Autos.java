package frc.robot.commands;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.ShooterSimple;
import frc.robot.util.Alliance;
import frc.robot.util.FieldPoseUtils;
import java.util.Set;

public class Autos {
  enum Trajectory {
    AmpToMiddle("Amp to Middle"),
    AmpToSource("Amp to Source"),
    FarSideToAmp("Far side to Amp"),
    FarSideToSource("Far side to Source");

    private final String fileName;

    Trajectory(String fileName) {
      this.fileName = fileName;
    }
  }

  private static Command followPath(Drive drive, Trajectory traj) {
    ChoreoTrajectory trajectory = Choreo.getTrajectory(traj.fileName);

    return new SequentialCommandGroup(
        new DriveToPose(
            drive,
            Alliance.isRed()
                ? FieldPoseUtils.flipPose(trajectory.getInitialPose())
                : trajectory.getInitialPose()),
        Choreo.choreoSwerveCommand(
            trajectory,
            drive::getPose,
            new PIDController(Constants.DrivePIDConstants.KpX, 0, 0),
            new PIDController(Constants.DrivePIDConstants.KpY, 0, 0),
            new PIDController(Constants.DrivePIDConstants.KpTheta, 0, 0),
            drive::runVelocity,
            Alliance::isRed));
  }

  public static Command CloseSideToAmp(Drive drive, ShooterSimple shooter) {
    return new DeferredCommand(
        () ->
            new SequentialCommandGroup(
                new DriveToPose(drive, FieldPoseUtils.alignedWithAmpPose()), ShootInAmp(shooter)),
        Set.of(drive));
  }

  public static Command FarSideToAmp(Drive drive, ShooterSimple shooter) {
    return new DeferredCommand(
        () ->
            new SequentialCommandGroup(
                followPath(drive, Trajectory.FarSideToAmp),
                new DriveToPose(drive, FieldPoseUtils.alignedWithAmpPose()),
                ShootInAmp(shooter)),
        Set.of(drive));
  }

  public static Command CloseSideToAmpToSource(Drive drive, ShooterSimple shooter) {
    return new DeferredCommand(
        () ->
            new SequentialCommandGroup(
                new DriveToPose(drive, FieldPoseUtils.alignedWithAmpPose()),
                ShootInAmp(shooter),
                followPath(drive, Trajectory.AmpToSource),
                new DriveToPose(drive, FieldPoseUtils.alignedWithSourcePose())),
        Set.of(drive));
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
        Set.of(drive));
  }

  public static Command CloseSideToAmpToMiddle(Drive drive, ShooterSimple shooter) {
    return new DeferredCommand(
        () ->
            new SequentialCommandGroup(
                new DriveToPose(drive, FieldPoseUtils.alignedWithAmpPose()),
                ShootInAmp(shooter),
                followPath(drive, Trajectory.AmpToMiddle)),
        Set.of(drive));
  }

  public static Command FarSideToAmpToMiddle(Drive drive, ShooterSimple shooter) {
    return new DeferredCommand(
        () ->
            new SequentialCommandGroup(
                followPath(drive, Trajectory.FarSideToAmp),
                new DriveToPose(drive, FieldPoseUtils.alignedWithAmpPose()),
                ShootInAmp(shooter),
                followPath(drive, Trajectory.AmpToMiddle)),
        Set.of(drive));
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
    return new RunCommand(() -> shooter.setFlywheels(0.3, -0.3)).withTimeout(2);
  }
}
