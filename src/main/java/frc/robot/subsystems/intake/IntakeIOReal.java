package frc.robot.subsystems.intake;

import static edu.wpi.first.math.util.Units.rotationsToRadians;
import static frc.robot.constants.IdConstants.CANId.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.constants.IdConstants;
import frc.robot.constants.IntakeConstants;

public class IntakeIOReal implements IntakeIO {
  private final CANSparkMax rightExtensionMotor =
      new CANSparkMax(RightIntakeExtensionMotorId, CANSparkMax.MotorType.kBrushless);
  private final CANSparkMax leftExtensionMotor =
      new CANSparkMax(LeftIntakeExtensionMotorId, CANSparkMax.MotorType.kBrushless);
  private final VictorSPX articulationMotor = new VictorSPX(IntakeArticulationMotorId);
  private final VictorSPX spinMotor = new VictorSPX(IntakeSpinMotorId);
  private final DutyCycleEncoder articulationEncoder;

  private final RelativeEncoder rightExtensionEncoder;
  private final RelativeEncoder leftExtensionEncoder;

  public IntakeIOReal() {
    leftExtensionMotor.restoreFactoryDefaults();
    rightExtensionMotor.restoreFactoryDefaults();

    rightExtensionEncoder = rightExtensionMotor.getEncoder();
    leftExtensionEncoder = leftExtensionMotor.getEncoder();
    articulationEncoder = new DutyCycleEncoder(IdConstants.DIOId.IntakeArticulationEncoderId);

    rightExtensionMotor.setInverted(true);
    leftExtensionMotor.setInverted(true);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.rightExtensionPositionRad = rotationsToRadians(rightExtensionEncoder.getPosition());
    inputs.rightExtensionVelocityRadPerSec =
        rotationsToRadians(rightExtensionEncoder.getVelocity());
    inputs.rightExtensionAppliedVolts =
        rightExtensionMotor.getAppliedOutput() * rightExtensionMotor.getBusVoltage();
    inputs.rightExtensionCurrentAmps = rightExtensionMotor.getOutputCurrent();

    inputs.leftExtensionPositionRad = rotationsToRadians(leftExtensionEncoder.getPosition());
    inputs.leftExtensionVelocityRadPerSec = rotationsToRadians(leftExtensionEncoder.getVelocity());
    inputs.leftExtensionAppliedVolts =
        leftExtensionMotor.getAppliedOutput() * leftExtensionMotor.getBusVoltage();
    inputs.leftExtensionCurrentAmps = leftExtensionMotor.getOutputCurrent();

    inputs.articulationPosition =
        Rotation2d.fromRotations(
            articulationEncoder.getAbsolutePosition()
                + IntakeConstants.AbsoluteEncoderOffset); // TODO: set offset
    inputs.articulationAppliedVolts = articulationMotor.getMotorOutputVoltage();

    inputs.spinAppliedVolts = spinMotor.getMotorOutputVoltage();
  }

  @Override
  public void extensionRunVoltage(double left, double right) {
    leftExtensionMotor.setVoltage(left);
    rightExtensionMotor.setVoltage(right);
  }

  @Override
  public void articulationRunVoltage(double voltage) {
    articulationMotor.set(
        VictorSPXControlMode.PercentOutput, /*voltage / articulationMotor.getBusVoltage()*/ 0);
  }

  @Override
  public void spinRunVoltage(double voltage) {
    spinMotor.set(ControlMode.PercentOutput, voltage / articulationMotor.getBusVoltage());
  }
}
