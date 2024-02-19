package frc.robot.constants;

public enum Trajectory {
  AmpToMiddle("Amp to Middle (Week 0)"),
  AmpToSource("Amp to Source"),
  FarSideToAmp("Far side to Amp"),
  FarSideToSource("Far side to Source");

  public final String fileName;

  Trajectory(String fileName) {
    this.fileName = fileName;
  }
}
