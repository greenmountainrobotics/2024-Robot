package frc.robot.subsystems.apriltagvision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

public class AprilTagVision extends SubsystemBase {
  private final AprilTagProvider[] implementations;

  public AprilTagVision(AprilTagProvider... implementations) {
    this.implementations = implementations;
  }

  @Override
  public void periodic() {
    for (AprilTagProvider implementation : implementations) {
      implementation.periodic();
    }
  }

  public void setDataInterface(
      BiConsumer<Pose2d, Double> poseConsumer, Supplier<Pose2d> referencePoseSupplier) {
    for (AprilTagProvider implementation : implementations) {
      implementation.setDataInterface(poseConsumer, referencePoseSupplier);
    }
  }
}
