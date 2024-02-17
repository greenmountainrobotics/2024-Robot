package frc.robot.subsystems.apriltagvision;

import static org.photonvision.PhotonPoseEstimator.PoseStrategy.*;

import org.photonvision.PhotonCamera;

public class PhotonVisionIOPhotonVision implements PhotonVisionIO {
  private final PhotonCamera camera;
  private final String cameraName;

  public PhotonVisionIOPhotonVision(String cameraName) {
    camera = new PhotonCamera(cameraName);
    this.cameraName = cameraName;
  }

  @Override
  public void updateInputs(AprilTagIOInputs inputs) {
    inputs.isConnected = camera.isConnected();
    inputs.latestResult = camera.getLatestResult();
  }

  @Override
  public void updateCameraName(AprilTagIOInputs inputs) {
    inputs.cameraName = cameraName;
  }
}
