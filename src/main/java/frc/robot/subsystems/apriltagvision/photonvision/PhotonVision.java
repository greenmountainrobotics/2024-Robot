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

  private Pose3d estimatedPose = new Pose3d();

  private PhotonPoseEstimator photonPoseEstimator;

  private Pose3d[] targetPoses = new Pose3d[] {};
  private String cameraName;

  public PhotonVision(PhotonVisionIO io) {
    this.io = io;
    aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    if (cameraName == null) cameraName = inputs.camera;
    Logger.processInputs("PhotonVision/" + cameraName, inputs);

    if (photonPoseEstimator == null) {
      photonPoseEstimator =
          new PhotonPoseEstimator(
              aprilTagFieldLayout, MULTI_TAG_PNP_ON_COPROCESSOR, inputs.robotToCam);
      photonPoseEstimator.setMultiTagFallbackStrategy(CLOSEST_TO_REFERENCE_POSE);
    }

    Pose2d referencePose = referencePoseSupplier.get();
    Pose3d referencePose3d =
        new Pose3d(
            referencePose.getX(),
            referencePose.getY(),
            0,
            new Rotation3d(0, 0, referencePose.getRotation().getRadians()));

    Logger.recordOutput(
        "PhotonVision/" + cameraName + "/CameraPose", referencePose3d.plus(inputs.robotToCam));

    Logger.recordOutput("PhotonVision/" + cameraName + "/robotToCam", inputs.robotToCam);

    photonPoseEstimator.setReferencePose(referencePoseSupplier.get());

    inputs.latestResult.setTimestampSeconds(inputs.timestamp);
    var update = photonPoseEstimator.update(inputs.latestResult);
    Logger.recordOutput("PhotonVision/" + cameraName + "/LatestResult", inputs.latestResult);
    Logger.recordOutput("PhotonVision/" + cameraName + "/isPresent", update.isPresent());
    Logger.recordOutput(
        "PhotonVision" + cameraName + "/Best",
        inputs.latestResult.getMultiTagResult().estimatedPose.best);

    if (update.isPresent()) {
      estimatedPose = update.get().estimatedPose;
      if (estimatedPose.getZ() > 0.1) return;
      poseConsumer.accept(estimatedPose.toPose2d(), update.get().timestampSeconds);

      targetPoses =
          update.get().targetsUsed.stream()
              .map(
                  target ->
                      referencePose3d.plus(inputs.robotToCam).plus(target.getBestCameraToTarget()))
              .toArray(Pose3d[]::new);
    } else {
      targetPoses = new Pose3d[] {};
    }

    Logger.recordOutput("PhotonVision/" + cameraName + "/TargetPoses", targetPoses);
    Logger.recordOutput("PhotonVision/" + cameraName + "/Pose2d", estimatedPose.toPose2d());
    Logger.recordOutput("PhotonVision/" + cameraName + "/Pose3d", estimatedPose);
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
