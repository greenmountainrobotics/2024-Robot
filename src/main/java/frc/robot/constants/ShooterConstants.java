package frc.robot.constants;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;

public class ShooterConstants {
  public static double PivotHeight = Units.inchesToMeters(10.197);
  public static double PivotX = Units.inchesToMeters(0);

  public static final double VelocityToleranceCoefficient = 0.02;
  public static final double ArticulationToleranceRad = 0.02;

  public static final double ShootingVelocityRPM = 4000;

  /** k: m, v: rad */
  public static final InterpolatingDoubleTreeMap ArticulationMap = new InterpolatingDoubleTreeMap();

  static {
    ArticulationMap.put(1.0, Math.PI / 2);
    ArticulationMap.put(2.0, Math.PI / 4);
    ArticulationMap.put(5.0, Math.PI);
  }
}
