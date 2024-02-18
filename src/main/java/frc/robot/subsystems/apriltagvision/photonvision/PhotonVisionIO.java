package frc.robot.subsystems.apriltagvision.photonvision;

import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLog;
import org.photonvision.targeting.PhotonPipelineResult;

public interface PhotonVisionIO {
  @AutoLog
  class PhotonVisionIOInputs {
    public boolean isConnected;
    public PhotonPipelineResult latestResult = new PhotonPipelineResult();
    public Constants.Camera camera;
  }

  default void updateInputs(PhotonVisionIOInputs inputs) {}

  default void updateCamera(PhotonVisionIOInputs inputs) {}
}
