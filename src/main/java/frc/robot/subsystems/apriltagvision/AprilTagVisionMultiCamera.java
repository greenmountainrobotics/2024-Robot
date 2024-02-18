package frc.robot.subsystems.apriltagvision;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

public class AprilTagVisionMultiCamera implements AprilTagVision {
  private final AprilTagVision[] implementations;

  public AprilTagVisionMultiCamera(AprilTagVision... implementations) {
    this.implementations = implementations;
  }

  @Override
  public void periodic() {
    for (AprilTagVision implementation : implementations) {
      implementation.periodic();
    }
  }

  @Override
  public void setDataInterface(
      BiConsumer<Pose2d, Double> poseConsumer, Supplier<Pose2d> referencePoseSupplier) {
    for (AprilTagVision implementation : implementations) {
      implementation.setDataInterface(poseConsumer, referencePoseSupplier);
    }
  }
}
