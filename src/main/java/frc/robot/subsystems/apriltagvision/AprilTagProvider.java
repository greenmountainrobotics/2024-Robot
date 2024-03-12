package frc.robot.subsystems.apriltagvision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

public interface AprilTagProvider {
  void setDataInterface(
      BiConsumer<Pose2d, Double> poseConsumer, Supplier<Pose2d> referencePoseSupplier);

  void periodic();

  Pose3d[] getTargetPoses();

  boolean isConnected();
}
