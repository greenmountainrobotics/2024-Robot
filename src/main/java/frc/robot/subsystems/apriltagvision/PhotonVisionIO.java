package frc.robot.subsystems.apriltagvision;

import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLog;
import org.photonvision.targeting.PhotonPipelineResult;

public interface PhotonVisionIO {
  @AutoLog
  class AprilTagIOInputs {
    public boolean isConnected;
    public PhotonPipelineResult latestResult = new PhotonPipelineResult();
    public Constants.Camera camera;
  }

  default void updateInputs(AprilTagIOInputs inputs) {}

  default void updateCamera(AprilTagIOInputs inputs) {}
}
