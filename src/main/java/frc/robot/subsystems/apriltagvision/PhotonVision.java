package frc.robot.subsystems.apriltagvision;

import static edu.wpi.first.math.util.Units.degreesToRadians;
import static edu.wpi.first.math.util.Units.inchesToMeters;
import static org.photonvision.PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE;
import static org.photonvision.PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import java.util.function.BiConsumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonPoseEstimator;

public class PhotonVision extends AprilTagVision {
  private PhotonVisionIO io;
  private AprilTagIOInputsAutoLogged inputs = new AprilTagIOInputsAutoLogged();
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
    io.updateCameraName(inputs);
    processInputs();

    switch (inputs.cameraName) {
      case "Arducam_OV2311_USB_Camera" -> robotToCam =
          new Transform3d(
              new Translation3d(0, inchesToMeters(-1.260), inchesToMeters(7.940)),
              new Rotation3d(0, degreesToRadians(180 + 65 + 90), Math.PI));
      default -> robotToCam =
          new Transform3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0.0, 0.0, 0.0));
    }

    aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    photonPoseEstimator =
        new PhotonPoseEstimator(aprilTagFieldLayout, MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam);
    photonPoseEstimator.setMultiTagFallbackStrategy(CLOSEST_TO_REFERENCE_POSE);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    processInputs();
    Logger.recordOutput("PhotonVision/Pose2d", estimatedPose.toPose2d());
    Logger.recordOutput("PhotonVision/Pose3d", estimatedPose);

    photonPoseEstimator.setReferencePose(referencePoseSupplier.get());

    photonPoseEstimator
        .update()
        .ifPresent(
            estimated -> {
              estimatedPose = estimated.estimatedPose;
              poseConsumer.accept(estimatedPose.toPose2d(), estimated.timestampSeconds);
            });
  }

  public void setDataInterface(
      BiConsumer<Pose2d, Double> poseConsumer, Supplier<Pose2d> referencePoseSupplier) {
    this.poseConsumer = poseConsumer;
    this.referencePoseSupplier = referencePoseSupplier;
  }
}
