// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static edu.wpi.first.math.util.Units.inchesToMeters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class CANIdConstants {
    public static final int FrontLeftDriveId = 1;
    public static final int FrontLeftTurnId = 2;
    public static final int FrontLeftEncoderId = 3;

    public static final int FrontRightDriveId = 4;
    public static final int FrontRightTurnId = 5;
    public static final int FrontRightEncoderId = 6;

    public static final int BackLeftDriveId = 7;
    public static final int BackLeftTurnId = 8;
    public static final int BackLeftEncoderId = 9;

    public static final int BackRightDriveId = 10;
    public static final int BackRightTurnId = 11;
    public static final int BackRightEncoderId = 12;

    public static final int TopSpinMotorId = 21;
    public static final int BottomSpinMotorId = 22;

    public static final int PigeonId = 31;
    public static final int RevPDHId = 32;
  }

  public static final class PWMIdConstants {
    public static final int LedsId = 0;
  }

  public enum Battery {
    CIABATTA(2023),
    CLUNKY_NOISES(2023),
    SOURDOUGH(2024),
    CHALLAH(2023),
    RYE(2023),
    NONE(0);

    public final int year;

    Battery(int year) {
      this.year = year;
    }
  }

  public static final class FieldConstants {
    public static final double FieldWidth =
        inchesToMeters(
            76.1 * 2 // 2 * width of amp
                + 250.50 * 2); // 2 * width between centerline and source
    public static final double FieldHeight = inchesToMeters(323.00);

    public static final Translation2d AmpCenter =
        new Translation2d(inchesToMeters(72.5), FieldHeight);

    // facing out
    public static final Rotation2d AmpRotation = Rotation2d.fromDegrees(-90);

    public static final Translation2d SourceCloseSideCorner =
        new Translation2d(
            FieldWidth - inchesToMeters(76.1), // field width - width of amp
            0.0 // on bottom edge of field
            );
    public static final Translation2d SourceFarSideCorner =
        new Translation2d(
            FieldWidth, inchesToMeters(60.75 - 18.0)); // height from edge - height of wall

    // facing out
    public static final Rotation2d SourceRotation = Rotation2d.fromDegrees(120);

    public static final Translation2d SpeakerBottomCloseSideCorner =
        new Translation2d(0, SourceFarSideCorner.getY() + inchesToMeters(133.677));

    public static final Translation2d SpeakerTopCloseSideCorner =
        new Translation2d(0, SpeakerBottomCloseSideCorner.getY() + inchesToMeters(82.400));

    private static final Translation2d SpeakerCloseCornerToFarCorner =
        new Translation2d(inchesToMeters(35.694542), inchesToMeters(20.824760));

    public static final Translation2d SpeakerBottomFarSideCorner =
        SpeakerBottomCloseSideCorner.plus(SpeakerCloseCornerToFarCorner);

    public static final Translation2d SpeakerTopFarSideCorner =
        SpeakerTopCloseSideCorner.plus(
            new Translation2d(
                -SpeakerCloseCornerToFarCorner.getX(), SpeakerCloseCornerToFarCorner.getY()));

    public static final Rotation2d SpeakerRotation = new Rotation2d();
  }

  public static final class RobotConstants {
    public static final double TrackWidthY;
    public static final double TrackWidthX;
    public static final double WidthWithBumpersY;
    public static final double WidthWithBumpersX;

    static {
      switch (robot) {
        case MAIN_2024, SIMBOT -> {
          TrackWidthY = inchesToMeters(22.75);
          TrackWidthX = inchesToMeters(22.75);
          WidthWithBumpersY = inchesToMeters(30.75);
          WidthWithBumpersX = inchesToMeters(30.75);
        }
        default -> {
          TrackWidthY = 0;
          TrackWidthX = 0;
          WidthWithBumpersY = 0;
          WidthWithBumpersX = 0;
        }
      }
    }
  }

  public static final class DrivePIDConstants {
    public static final double KpX = 3.0;
    public static final double KpY = 3.0;
    public static final double KpTranslation = 3.0;
    public static final double KpTheta = 5.0;
  }

  private static final RobotType robot = RobotType.MAIN_2024;

  public enum RobotType {
    SIMBOT,
    MAIN_2024
  }

  public enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static Mode getMode() {
    if (robot == RobotType.SIMBOT) return Mode.SIM;
    return RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
  }

  /** Checks whether the robot the correct robot is selected when deploying. */
  public static void main(String... args) {
    if (robot == RobotType.SIMBOT) {
      System.err.println("Cannot deploy, invalid robot selected: " + robot.toString());
      System.exit(1);
    }
  }
}
