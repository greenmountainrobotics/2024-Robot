package frc.robot.subsystems.apriltagvision.photonvision;

import frc.robot.Constants;
import org.photonvision.PhotonCamera;

public class PhotonVisionIOReal implements PhotonVisionIO {
  private final PhotonCamera photonCamera;
  private final Constants.Camera camera;

  public PhotonVisionIOReal(Constants.Camera camera) {
    photonCamera = new PhotonCamera(camera.name);
    this.camera = camera;
  }

  @Override
  public void updateInputs(PhotonVisionIOInputs inputs) {
    inputs.isConnected = photonCamera.isConnected();
    inputs.latestResult = photonCamera.getLatestResult();
    inputs.camera = camera;
  }

  @Override
  public void updateCamera(PhotonVisionIOInputs inputs) {
    inputs.camera = camera;
  }
}
