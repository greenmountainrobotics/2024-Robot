package frc.robot.subsystems.apriltagvision;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.function.BiConsumer;
import org.littletonrobotics.junction.Logger;

public class PhotonVision extends AprilTagVision {
  private PhotonVisionIO io;
  private AprilTagIOInputsAutoLogged inputs = new AprilTagIOInputsAutoLogged();
  private BiConsumer<Pose2d, Double> poseConsumer = (x, y) -> {};

  public PhotonVision(PhotonVisionIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("PhotonVision", inputs);

    if (inputs.latestTimestamp != 0.0)
      poseConsumer.accept(inputs.estimatedPose.toPose2d(), inputs.latestTimestamp);
  }

  public void setDataInterface(BiConsumer<Pose2d, Double> poseConsumer) {
    this.poseConsumer = poseConsumer;
  }
}
