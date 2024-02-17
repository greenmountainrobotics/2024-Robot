package frc.robot.subsystems.apriltagvision;

import org.littletonrobotics.junction.AutoLog;
import org.photonvision.targeting.PhotonPipelineResult;

public interface PhotonVisionIO {
  @AutoLog
  class AprilTagIOInputs {
    public boolean isConnected;
    public PhotonPipelineResult latestResult = new PhotonPipelineResult();
    public String cameraName = "";
  }

  default void updateInputs(AprilTagIOInputs inputs) {}

  default void updateCameraName(AprilTagIOInputs inputs) {}
}
