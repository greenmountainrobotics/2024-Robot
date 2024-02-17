package frc.robot.subsystems.apriltagvision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

public abstract class AprilTagVision extends SubsystemBase {
  public abstract void setDataInterface(
      BiConsumer<Pose2d, Double> poseConsumer, Supplier<Pose2d> referencePoseSupplier);
}
