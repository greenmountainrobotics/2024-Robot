package frc.robot.subsystems.apriltagvision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import org.littletonrobotics.junction.AutoLog;

public interface PhotonVisionIO {
  @AutoLog
  public static class AprilTagIOInputs {
    public boolean isConnected;
    public Pose3d estimatedPose = new Pose3d();
    public double latestTimestamp;
    public Transform3d[] robotToTargetList = new Transform3d[0];
  }

  public default void updateInputs(AprilTagIOInputs inputs) {}
}
