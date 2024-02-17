package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class ShooterIOSparkMax implements ShooterIO {
  private CANSparkMax topSpinMotor =
      new CANSparkMax(
          Constants.CANIdConstants.TopSpinMotorId, CANSparkLowLevel.MotorType.kBrushless);
  private CANSparkMax bottomSpinMotor =
      new CANSparkMax(
          Constants.CANIdConstants.BottomSpinMotorId, CANSparkLowLevel.MotorType.kBrushless);
  // private CANSparkMax pivotMotor = new CANSparkMax(3, CANSparkLowLevel.MotorType.kBrushed);

  private final RelativeEncoder topEncoder;
  private final RelativeEncoder bottomEncoder;

  // private final RelativeEncoder pivotEncoder;

  public ShooterIOSparkMax() {
    topEncoder = topSpinMotor.getEncoder();
    bottomEncoder = bottomSpinMotor.getEncoder();
    topSpinMotor.setInverted(true);
    // pivotEncoder = pivotMotor.getEncoder();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.bottomPositionRad = Units.rotationsToRadians(bottomEncoder.getPosition());
    inputs.topPositionRad = Units.rotationsToRadians(topEncoder.getPosition());
    // inputs.pivotPosition = Rotation2d.fromRotations(topEncoder.getPosition());

    inputs.bottomVelocityRadPerSec = Units.rotationsToRadians(bottomEncoder.getVelocity());
    inputs.topVelocityRadPerSec = Units.rotationsToRadians(topEncoder.getVelocity());
    // inputs.pivotVelocityRadPerSec = Units.rotationsToRadians(pivotEncoder.getVelocity());

    inputs.bottomAppliedVolts =
        bottomSpinMotor.getAppliedOutput() * bottomSpinMotor.getBusVoltage();
    inputs.topAppliedVolts = topSpinMotor.getAppliedOutput() * topSpinMotor.getBusVoltage();
    // inputs.pivotAppliedVolts = pivotMotor.getAppliedOutput() * pivotMotor.getBusVoltage();
  }

  /*    @Override
  public void setPivotVoltage(double volts) {
      pivotMotor.setVoltage(volts);
  }*/

  @Override
  public void setTopVoltage(double volts) {
    topSpinMotor.setVoltage(volts);
  }

  @Override
  public void setBottomVoltage(double volts) {
    bottomSpinMotor.setVoltage(volts);
  }
}
