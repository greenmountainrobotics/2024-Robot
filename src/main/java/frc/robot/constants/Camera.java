package frc.robot.constants;

import static edu.wpi.first.math.util.Units.degreesToRadians;
import static edu.wpi.first.math.util.Units.inchesToMeters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public enum Camera {
  BackCamera(
      "Arducam_OV2311_USB_Camera",
      1280,
      720,
      Rotation2d.fromDegrees(75),
      new Transform3d(
          new Translation3d(inchesToMeters(-2.091), inchesToMeters(-0.005), inchesToMeters(6.061)),
          new Rotation3d(0, degreesToRadians(-25), degreesToRadians(180)))),
  FrontLeftCamera(
      "FrontLeftCam",
      1280,
      720,
      Rotation2d.fromDegrees(70),
      new Transform3d(
          new Translation3d(inchesToMeters(6.894), inchesToMeters(11.382), inchesToMeters(10.621)),
          new Rotation3d(0, degreesToRadians(-25), degreesToRadians(60)))),
  FrontRightCamera(
      "Arducam_OV9281_USB_Camera",
      1280,
      720,
      Rotation2d.fromDegrees(70),
      new Transform3d(
          new Translation3d(inchesToMeters(6.894), inchesToMeters(-11.390), inchesToMeters(10.621)),
          new Rotation3d(0, degreesToRadians(-25), degreesToRadians(-60))));
  public final String name;
  public final Transform3d robotToCam;
  public final int width;
  public final int height;
  public final Rotation2d fov;

  Camera(String name, int width, int height, Rotation2d fov, Transform3d robotToCam) {
    this.name = name;
    this.robotToCam = robotToCam;
    this.width = width;
    this.height = height;
    this.fov = fov;
  }
}
