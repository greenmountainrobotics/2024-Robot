package frc.robot.subsystems.apriltagvision.photonvision;

import static org.photonvision.PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE;
import static org.photonvision.PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import frc.robot.subsystems.apriltagvision.AprilTagVision;
import java.util.function.BiConsumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonPoseEstimator;

public class PhotonVision implements AprilTagVision {
  private PhotonVisionIO io;
  private PhotonVisionIOInputsAutoLogged inputs = new PhotonVisionIOInputsAutoLogged();
  private BiConsumer<Pose2d, Double> poseConsumer = (x, y) -> {};
  private Supplier<Pose2d> referencePoseSupplier = () -> new Pose2d();
  private final AprilTagFieldLayout aprilTagFieldLayout;
  private final Transform3d robotToCam;

  private Pose3d estimatedPose = new Pose3d();

  private final PhotonPoseEstimator photonPoseEstimator;

  private void processInputs() {
    Logger.processInputs("PhotonVision", inputs);
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
    Logger.recordOutput("PhotonVision/" + inputs.camera + "/Pose2d", estimatedPose.toPose2d());
    Logger.recordOutput("PhotonVision/" + inputs.camera + "/Pose3d", estimatedPose);

    photonPoseEstimator.setReferencePose(referencePoseSupplier.get());

    photonPoseEstimator
        .update(inputs.latestResult)
        .ifPresent(
            estimated -> {
              estimatedPose = estimated.estimatedPose;
              poseConsumer.accept(estimatedPose.toPose2d(), estimated.timestampSeconds);
            });
  }

  @Override
  public void setDataInterface(
      BiConsumer<Pose2d, Double> poseConsumer, Supplier<Pose2d> referencePoseSupplier) {
    this.poseConsumer = poseConsumer;
    this.referencePoseSupplier = referencePoseSupplier;
  }
}
