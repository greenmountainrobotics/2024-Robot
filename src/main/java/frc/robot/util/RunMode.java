package frc.robot.util;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Config;

public enum RunMode {
  /** Running on a real robot. */
  REAL,

  /** Running a physics simulator. */
  SIM,

  /** Replaying from a log file. */
  REPLAY;

  public static RunMode getMode() {
    if (Config.SIMULATION) return RunMode.SIM;
    return RobotBase.isReal() ? RunMode.REAL : RunMode.REPLAY;
  }

  /** Checks whether the robot the correct robot is selected when deploying. */
  public static void main(String... args) {
    if (Config.SIMULATION) {
      System.err.println("Cannot deploy; Simulation mode enabled.");
      System.exit(1);
    }
  }
}
