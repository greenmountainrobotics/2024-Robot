package frc.robot.subsystems.apriltagvision.photonvision;

import edu.wpi.first.math.geometry.Transform3d;
import org.littletonrobotics.junction.AutoLog;
import org.photonvision.targeting.PhotonPipelineResult;

public interface PhotonVisionIO {
  @AutoLog
  class PhotonVisionIOInputs {
    public boolean isConnected;
    public PhotonPipelineResult latestResult = new PhotonPipelineResult();
    public String camera;

    public Transform3d robotToCam;
  }

  default void updateInputs(PhotonVisionIOInputs inputs) {}

  default void updateCamera(PhotonVisionIOInputs inputs) {}
}
