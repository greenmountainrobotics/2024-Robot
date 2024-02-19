package frc.robot.constants;

import static edu.wpi.first.math.util.Units.degreesToRadians;
import static edu.wpi.first.math.util.Units.inchesToMeters;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public enum Camera {
  BackCamera(
      "Arducam_OV2311_USB_Camera",
      new Transform3d(
          new Translation3d(0, inchesToMeters(1.260), inchesToMeters(7.940)),
          new Rotation3d(0, degreesToRadians(-25), Math.PI)));

  public final String name;
  public final Transform3d robotToCam;

  Camera(String name, Transform3d robotToCam) {
    this.name = name;
    this.robotToCam = robotToCam;
  }
}
