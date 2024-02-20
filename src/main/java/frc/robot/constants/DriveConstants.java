package frc.robot.constants;

import static edu.wpi.first.math.util.Units.inchesToMeters;

public final class DriveConstants {
  public static final double TrackWidthY = inchesToMeters(22.75);
  public static final double TrackWidthX = inchesToMeters(22.75);
  public static final double WidthWithBumpersY = inchesToMeters(30.75);
  public static final double WidthWithBumpersX = inchesToMeters(30.75);

  // Gear ratios for SDS MK4i L3
  public static final double DriveGearRatio = (50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0);
  public static final double TurnGearRatio = 150.0 / 7.0;

  public static final double FrontLeftEncoderOffset = -4.5345926536;
  public static final double FrontRightEncoderOffset = -2.326;
  public static final double BackLeftEncoderOffset = -3.8085926536;
  public static final double BackRightEncoderOffset = -0.907;
}
