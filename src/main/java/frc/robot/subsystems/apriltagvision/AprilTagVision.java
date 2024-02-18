package frc.robot.subsystems.apriltagvision;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

public interface AprilTagVision {
  void setDataInterface(
      BiConsumer<Pose2d, Double> poseConsumer, Supplier<Pose2d> referencePoseSupplier);

  void periodic();
}
