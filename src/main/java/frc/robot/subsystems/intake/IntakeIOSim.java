package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeIOSim implements IntakeIO {
  private static final double LOOP_PERIOD_SECS = 0.02;
  private final DCMotorSim leftExtensionMotor = new DCMotorSim(DCMotor.getNEO(1), 1, 0.001);
  private final DCMotorSim rightExtensionMotor = new DCMotorSim(DCMotor.getNEO(1), 1, 0.001);

  private final DCMotorSim articulationMotor = new DCMotorSim(DCMotor.getVex775Pro(1), 1, 0.001);
  private final DCMotorSim spinMotor = new DCMotorSim(DCMotor.getVex775Pro(1), 1, 0.001);

  private double leftExtensionMotorAppliedVolts = 0.0;
  private double rightExtensionMotorAppliedVolts = 0.0;

  private double articulationMotorAppliedVolts = 0.0;
  private double spinMotorAppliedVolts = 0.0;

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    rightExtensionMotor.update(LOOP_PERIOD_SECS);
    leftExtensionMotor.update(LOOP_PERIOD_SECS);
    articulationMotor.update(LOOP_PERIOD_SECS);
    spinMotor.update(LOOP_PERIOD_SECS);

    inputs.rightExtensionPositionRad = rightExtensionMotor.getAngularPositionRad();
    inputs.rightExtensionVelocityRadPerSec = rightExtensionMotor.getAngularVelocityRadPerSec();
    inputs.rightExtensionAppliedVolts = rightExtensionMotorAppliedVolts;
    inputs.rightExtensionCurrentAmps = rightExtensionMotor.getCurrentDrawAmps();

    inputs.leftExtensionPositionRad = leftExtensionMotor.getAngularPositionRad();
    inputs.leftExtensionVelocityRadPerSec = leftExtensionMotor.getAngularVelocityRadPerSec();
    inputs.leftExtensionAppliedVolts = leftExtensionMotorAppliedVolts;
    inputs.leftExtensionCurrentAmps = leftExtensionMotor.getCurrentDrawAmps();

    inputs.articulationPosition = Rotation2d.fromRadians(articulationMotor.getAngularPositionRad());
    inputs.articulationAppliedVolts = articulationMotorAppliedVolts;

    inputs.spinAppliedVolts = spinMotorAppliedVolts;
    inputs.spinCurrentAmps = spinMotor.getCurrentDrawAmps();
  }

  @Override
  public void extensionRunVoltage(double voltage) {
    leftExtensionMotorAppliedVolts = voltage;
    leftExtensionMotor.setInputVoltage(voltage);

    rightExtensionMotorAppliedVolts = voltage;
    leftExtensionMotor.setInputVoltage(voltage);
  }

  @Override
  public void articulationRunVoltage(double voltage) {
    articulationMotorAppliedVolts = voltage;
    articulationMotor.setInputVoltage(voltage);
  }

  @Override
  public void spinRunVoltage(double voltage) {
    spinMotorAppliedVolts = voltage;
    spinMotor.setInputVoltage(voltage);
  }
}
