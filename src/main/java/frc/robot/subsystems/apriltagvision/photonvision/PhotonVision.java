package frc.robot.subsystems.apriltagvision.photonvision;

import static org.photonvision.PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE;
import static org.photonvision.PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import frc.robot.subsystems.apriltagvision.AprilTagProvider;
import java.util.function.BiConsumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonPoseEstimator;

public class PhotonVision implements AprilTagProvider {
  private PhotonVisionIO io;
  private PhotonVisionIOInputsAutoLogged inputs = new PhotonVisionIOInputsAutoLogged();
  private BiConsumer<Pose2d, Double> poseConsumer = (x, y) -> {};
  private Supplier<Pose2d> referencePoseSupplier = () -> new Pose2d();
  private final AprilTagFieldLayout aprilTagFieldLayout;
  private final Transform3d robotToCam;

  private Pose3d estimatedPose = new Pose3d();

  private final PhotonPoseEstimator photonPoseEstimator;

  private Pose3d[] targetPoses = new Pose3d[] {};

  private void processInputs() {
    Logger.processInputs("PhotonVision/" + inputs.camera, inputs);
  }

  public PhotonVision(PhotonVisionIO io) {
    this.io = io;
    io.updateCamera(inputs);
    processInputs();

    robotToCam = inputs.camera.robotToCam;

    aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    photonPoseEstimator =
        new PhotonPoseEstimator(aprilTagFieldLayout, MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam);
    photonPoseEstimator.setMultiTagFallbackStrategy(CLOSEST_TO_REFERENCE_POSE);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    processInputs();

    Pose2d referencePose = referencePoseSupplier.get();
    Pose3d referencePose3d =
        new Pose3d(
            referencePose.getX(),
            referencePose.getY(),
            0,
            new Rotation3d(0, 0, referencePose.getRotation().getRadians()));

    Logger.recordOutput("PhotonVision/" + inputs.camera + "/Pose2d", estimatedPose.toPose2d());
    Logger.recordOutput("PhotonVision/" + inputs.camera + "/Pose3d", estimatedPose);
    Logger.recordOutput(
        "PhotonVision/" + inputs.camera + "/CameraPose",
        referencePose3d.plus(inputs.camera.robotToCam));

    photonPoseEstimator.setReferencePose(referencePoseSupplier.get());

    var update = photonPoseEstimator.update(inputs.latestResult);

    if (update.isPresent()) {
      estimatedPose = update.get().estimatedPose;
      poseConsumer.accept(estimatedPose.toPose2d(), update.get().timestampSeconds);

      targetPoses =
          update.get().targetsUsed.stream()
              .map(target -> referencePose3d.plus(robotToCam).plus(target.getBestCameraToTarget()))
              .toArray(Pose3d[]::new);
    } else {
      targetPoses = new Pose3d[] {};
    }

    Logger.recordOutput("PhotonVision/" + inputs.camera + "/TargetPoses", targetPoses);
  }

  public Pose3d[] getTargetPoses() {
    return targetPoses;
  }

  @Override
  public void setDataInterface(
      BiConsumer<Pose2d, Double> poseConsumer, Supplier<Pose2d> referencePoseSupplier) {
    this.poseConsumer = poseConsumer;
    this.referencePoseSupplier = referencePoseSupplier;
  }

  @Override
  public boolean isConnected() {
    return inputs.isConnected;
  }
}
