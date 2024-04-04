package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  class ClimberIOInputs {
    public double leftOutput;
    public double rightOutput;
  }

  default void updateInputs(ClimberIOInputs inputs) {}

  default void setLeftVoltage(double voltage) {}

  default void setRightVoltage(double voltage) {}
}
