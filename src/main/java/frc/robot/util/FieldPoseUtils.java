package frc.robot.util;

import static frc.robot.Constants.FieldConstants.*;
import static frc.robot.Constants.FieldConstants.SourceRotation;
import static frc.robot.Constants.RobotConstants.WidthWithBumpersX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class FieldPoseUtils {
  public static Pose2d flipPose(Pose2d pose) {
    return new Pose2d(
        FieldWidth - pose.getX(),
        pose.getY(),
        pose.getRotation().plus(Rotation2d.fromDegrees(180)).times(-1));
  }

  public static Pose2d alignedWithSourcePose() {
    Pose2d pose =
        new Pose2d(
            SourceCloseSideCorner.plus(SourceFarSideCorner)
                .div(2)
                .plus(new Translation2d(WidthWithBumpersX, 0).times(0.5).rotateBy(SourceRotation)),
            SourceRotation.plus(Rotation2d.fromDegrees(180)));
    if (Alliance.isRed()) pose = FieldPoseUtils.flipPose(pose);
    return pose;
  }

  public static Pose2d alignedWithAmpPose() {
    Pose2d pose =
        new Pose2d(
            AmpCenter.minus(
                new Translation2d(WidthWithBumpersX, 0)
                    .times(0.5)
                    .rotateBy(Rotation2d.fromDegrees(90))),
            AmpRotation.plus(Rotation2d.fromDegrees(180)));
    if (Alliance.isRed()) pose = FieldPoseUtils.flipPose(pose);
    return pose;
  }
}
