package frc.robot.subsystems.apriltagvision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.ArrayList;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

public class PhotonVisionIOPhotonVision implements PhotonVisionIO {
  private final AprilTagFieldLayout aprilTagFieldLayout;
  private final PhotonCamera camera;
  private final PhotonPoseEstimator photonPoseEstimator;
  private final Transform3d robotToCam;

  public PhotonVisionIOPhotonVision(String cameraName) {
    aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    PhotonPoseEstimator.PoseStrategy poseStrategy =
        PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

    switch (cameraName) {
      case "camera1":
        camera = new PhotonCamera("camera1");
        robotToCam =
            new Transform3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0.0, 0.0, 0.0));
        photonPoseEstimator =
            new PhotonPoseEstimator(aprilTagFieldLayout, poseStrategy, camera, robotToCam);
        break;
      default:
        camera = new PhotonCamera("photonCamera");
        robotToCam =
            new Transform3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0.0, 0.0, 0.0));
        photonPoseEstimator =
            new PhotonPoseEstimator(aprilTagFieldLayout, poseStrategy, camera, robotToCam);
    }
  }

  @Override
  public void updateInputs(AprilTagIOInputs inputs) {
    inputs.isConnected = camera.isConnected();

    photonPoseEstimator
        .update()
        .ifPresent(
            estimatedRobotPose -> {
              inputs.estimatedPose = estimatedRobotPose.estimatedPose;
              inputs.latestTimestamp = estimatedRobotPose.timestampSeconds;

              ArrayList<Transform3d> bestRobotToTargetList = new ArrayList<>();
              estimatedRobotPose.targetsUsed.forEach(
                  photonTrackedTarget -> {
                    bestRobotToTargetList.add(
                        robotToCam.plus(photonTrackedTarget.getBestCameraToTarget()));
                  });
              inputs.robotToTargetList = bestRobotToTargetList.toArray(new Transform3d[0]);
            });
  }
}
