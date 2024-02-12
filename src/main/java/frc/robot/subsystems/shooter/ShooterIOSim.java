package frc.robot.subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ShooterIOSim implements ShooterIO {

  private static final double LOOP_PERIOD_SECS = 0.02;

  // TODO: set constants
  private DCMotorSim topSpinMotor = new DCMotorSim(DCMotor.getNEO(1), 1, 0.025);
  private DCMotorSim bottomSpinMotor = new DCMotorSim(DCMotor.getNEO(1), 1, 0.025);
  /*
      private DCMotorSim pivotMotor = new DCMotorSim(DCMotor.getNEO(1), 1, 0.025);
  */

  private double topAppliedVolts = 0.0;
  private double bottomAppliedVolts = 0.0;

  /*
      private double pivotAppliedVolts = 0.0;
  */

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    topSpinMotor.update(LOOP_PERIOD_SECS);
    bottomSpinMotor.update(LOOP_PERIOD_SECS);
    /*
            pivotMotor.update(LOOP_PERIOD_SECS);
    */

    /*
            inputs.pivotPosition = new Rotation2d(pivotMotor.getAngularPositionRad());
            inputs.pivotVelocityRadPerSec = pivotMotor.getAngularVelocityRadPerSec();
            inputs.pivotAppliedVolts = pivotAppliedVolts;
            inputs.pivotCurrentAmps = new double[] {Math.abs(pivotMotor.getCurrentDrawAmps())};
    */

    inputs.topPositionRad = topSpinMotor.getAngularPositionRad();
    inputs.topVelocityRadPerSec = topSpinMotor.getAngularVelocityRadPerSec();
    inputs.topAppliedVolts = topAppliedVolts;
    inputs.topCurrentAmps = new double[] {Math.abs(topSpinMotor.getCurrentDrawAmps())};

    inputs.bottomPositionRad = bottomSpinMotor.getAngularPositionRad();
    inputs.bottomVelocityRadPerSec = bottomSpinMotor.getAngularVelocityRadPerSec();
    inputs.bottomAppliedVolts = bottomAppliedVolts;
    inputs.bottomCurrentAmps = new double[] {Math.abs(bottomSpinMotor.getCurrentDrawAmps())};
  }

  /*    @Override
  public void setPivotVoltage(double volts) {
      pivotAppliedVolts = volts;
      pivotMotor.setInputVoltage(volts);
  }*/

  @Override
  public void setTopVoltage(double volts) {
    topAppliedVolts = volts;
    topSpinMotor.setInputVoltage(volts);
  }

  @Override
  public void setBottomVoltage(double volts) {
    bottomAppliedVolts = volts;
    topSpinMotor.setInputVoltage(volts);
  }
}
