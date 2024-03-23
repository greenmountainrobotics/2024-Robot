package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;

public class ShooterConstants {
  public static double PivotHeight = Units.inchesToMeters(10.197);
  public static double PivotX = Units.inchesToMeters(0);

  public static final double VelocityToleranceRPM = 100;
  public static final double ArticulationToleranceRad = 0.02;

  public static final double ShootingVelocityRPM = 5500;
  public static final double ShooterGearRatio = 1.0 / 1.25;

  // measured from bottom edge of shooter facing forwards
  public static final Rotation2d AbsoluteEncoderOffset =
      Rotation2d.fromRadians(2.67).unaryMinus().plus(Rotation2d.fromRadians(Math.PI));

  /** k: m, v: rad */
  // UNUSED
  public static final InterpolatingDoubleTreeMap ArticulationMap = new InterpolatingDoubleTreeMap();

  static {
    ArticulationMap.put(1.0, Math.PI / 2);
    ArticulationMap.put(2.0, Math.PI / 4);
    ArticulationMap.put(5.0, Math.PI);
  }
}
