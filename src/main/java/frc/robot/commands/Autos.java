package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.constants.Trajectory;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.FieldPoseUtils;

public class Autos {

  public static Command CloseSideToAmp(Drive drive, Shooter shooter) {

    return new SequentialCommandGroup(
        drive.runToPose(FieldPoseUtils::alignedWithAmpPose), ShootInAmp(shooter));
  }

  public static Command FarSideToAmp(Drive drive, Shooter shooter) {
    return new SequentialCommandGroup(
        drive.followPath(Trajectory.FarSideToAmp),
        drive.runToPose(FieldPoseUtils::alignedWithAmpPose),
        ShootInAmp(shooter));
  }

  public static Command CloseSideToAmpToSource(Drive drive, Shooter shooter) {
    return new SequentialCommandGroup(
        drive.runToPose(FieldPoseUtils::alignedWithAmpPose),
        ShootInAmp(shooter),
        drive.followPath(Trajectory.AmpToSource),
        drive.runToPose(FieldPoseUtils::alignedWithSourcePose));
  }

  public static Command FarSideToAmpToSource(Drive drive, Shooter shooter) {
    return new SequentialCommandGroup(
        drive.followPath(Trajectory.FarSideToAmp),
        drive.runToPose(FieldPoseUtils::alignedWithAmpPose),
        ShootInAmp(shooter),
        drive.followPath(Trajectory.AmpToSource),
        drive.runToPose(FieldPoseUtils::alignedWithSourcePose));
  }

  public static Command CloseSideToAmpToMiddle(Drive drive, Shooter shooter) {
    return new SequentialCommandGroup(
        drive.runToPose(FieldPoseUtils::alignedWithAmpPose),
        ShootInAmp(shooter),
        drive.followPath(Trajectory.AmpToMiddle));
  }

  public static Command FarSideToAmpToMiddle(Drive drive, Shooter shooter) {
    return new SequentialCommandGroup(
        drive.followPath(Trajectory.FarSideToAmp),
        drive.runToPose(FieldPoseUtils::alignedWithAmpPose),
        ShootInAmp(shooter),
        drive.followPath(Trajectory.AmpToMiddle));
  }

  public static Command FarSideToSource(Drive drive) {
    return new SequentialCommandGroup(
        drive.followPath(Trajectory.FarSideToSource),
        drive.runToPose(FieldPoseUtils::alignedWithSourcePose));
  }

  public static Command ShootInAmp(Shooter shooter) {
    // TODO: No-op for now.
    return new InstantCommand(() -> {});
  }
}
