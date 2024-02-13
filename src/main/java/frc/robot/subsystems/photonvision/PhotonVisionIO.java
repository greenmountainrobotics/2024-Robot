package frc.robot.subsystems.photonvision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import org.littletonrobotics.junction.AutoLog;

import java.util.ArrayList;

public interface PhotonVisionIO {
  @AutoLog
  public static class AprilTagIOInputs {
    public boolean isConnected;
    public Pose3d estimatedPose;
    public double latestTimestamp;
    public ArrayList<Transform3d> robotToTargetList;
  }

  public default void updateInputs(AprilTagIOInputs inputs) {}

}
