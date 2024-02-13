package frc.robot.subsystems.apriltagvision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import org.littletonrobotics.junction.AutoLog;

public interface PhotonVisionIO {
  @AutoLog
  public static class AprilTagIOInputs {
    public boolean isConnected;
    public Pose3d estimatedPose;
    public double latestTimestamp;
    public Transform3d[] robotToTargetList;
  }

  public default void updateInputs(AprilTagIOInputs inputs) {}
}
