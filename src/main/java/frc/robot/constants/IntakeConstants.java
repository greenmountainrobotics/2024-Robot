package frc.robot.constants;

import static edu.wpi.first.math.util.Units.inchesToMeters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class IntakeConstants {

  public static final double MaxExtension = inchesToMeters(8.524);
  public static final double TargetExtension = inchesToMeters(7.693);
  public static final double MinExtension = inchesToMeters(2.625 + 0.5);
  public static final double StartRotatingDownwardsExtension = TargetExtension - inchesToMeters(6);

  public static final double ExtensionMetersPerRotation = 0.012;
  public static final double PivotHeight = inchesToMeters(8.369);
  public static final double ExtensionStartX = inchesToMeters(10.618500);

  // articulation is measured between extension and top side of end effector when retracted
  public static final Rotation2d TargetArticulation = Rotation2d.fromDegrees(-77);
  public static final Rotation2d RetractedArticulation = Rotation2d.fromDegrees(90 + 32.724839);
  public static final Rotation2d HalfwayArticulation = Rotation2d.fromDegrees(180 + 166.600512);
  public static final Rotation2d PointingAtShooterArticulation =
      Rotation2d.fromDegrees(180 - 52.993);
  public static final Rotation2d PointingUpArticulation = Rotation2d.fromDegrees(180 - 100.405);

  public static final double ExtensionTolerance = Units.inchesToMeters(0.5);
  public static final double ArticulationToleranceRad = 0.02;

  public static final double IntakeSpeed = -0.5;
  public static final double FeedIntoShooterSpeed = 0.5;
  public static final double ShootIntoAmpSpeed = 0.5;

  public static final double AbsoluteEncoderOffset = 0;
}
