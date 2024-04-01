package frc.robot.constants;

import static edu.wpi.first.math.util.Units.degreesToRadians;
import static edu.wpi.first.math.util.Units.inchesToMeters;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public enum Camera {
  BackCamera(
      "Arducam_OV2311_USB_Camera",
      1280,
      720,
      Rotation2d.fromDegrees(75),
      new Transform3d(
          new Translation3d(inchesToMeters(-2.091), inchesToMeters(-0.005), inchesToMeters(6.061)),
          new Rotation3d(0, degreesToRadians(-30), degreesToRadians(180))),
          VecBuilder.fill(0.9, 0.9, 0.9)),
  FrontLeftCamera(
      "FrontLeftCam",
      1280,
      720,
      Rotation2d.fromDegrees(70),
      new Transform3d(
          new Translation3d(inchesToMeters(6.894), inchesToMeters(11.382), inchesToMeters(10.621)),
          new Rotation3d(0, degreesToRadians(-25), degreesToRadians(60))),     VecBuilder.fill(0.9, 0.9, 0.9)),
  FrontRightCamera(
      "Arducam_OV9281_USB_Camera",
      1280,
      720,
      Rotation2d.fromDegrees(70),
      new Transform3d(
          new Translation3d(inchesToMeters(6.894), inchesToMeters(-11.390), inchesToMeters(10.621)),
          new Rotation3d(0, degreesToRadians(-25), degreesToRadians(-60))),     VecBuilder.fill(0.9, 0.9, 0.9));
  public final String name;
  public final Transform3d robotToCam;
  public final int width;
  public final int height;
  public final Rotation2d fov;
  public final Matrix<N3, N1> stddevs;

  Camera(String name, int width, int height, Rotation2d fov, Transform3d robotToCam, Matrix<N3, N1> stddevs) {
    this.name = name;
    this.robotToCam = robotToCam;
    this.width = width;
    this.height = height;
    this.fov = fov;
      this.stddevs = stddevs;
  }
}
