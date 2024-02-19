package frc.robot.constants;

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
