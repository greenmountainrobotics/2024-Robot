package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  class IntakeIOInputs {
    public double rightExtensionPositionRad = 0.0;
    public double rightExtensionVelocityRadPerSec = 0.0;
    public double rightExtensionAppliedVolts = 0.0;
    public double rightExtensionCurrentAmps = 0.0;

    public double leftExtensionPositionRad = 0.0;
    public double leftExtensionVelocityRadPerSec = 0.0;
    public double leftExtensionAppliedVolts = 0.0;
    public double leftExtensionCurrentAmps = 0.0;

    public Rotation2d articulationPosition = new Rotation2d();

    public double articulationAppliedVolts = 0.0;

    public double spinAppliedVolts = 0.0;
  }

  default void updateInputs(IntakeIOInputs inputs) {}

  default void extensionRunVoltage(double left, double right) {}

  default void articulationRunVoltage(double voltage) {}

  default void spinRunVoltage(double voltage) {}
}
