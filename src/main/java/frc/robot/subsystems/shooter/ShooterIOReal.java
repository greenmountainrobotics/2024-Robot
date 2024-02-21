package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.IdConstants;

public class ShooterIOReal implements ShooterIO {
  private final CANSparkMax topSpinMotor =
      new CANSparkMax(IdConstants.CANId.TopSpinMotorId, CANSparkLowLevel.MotorType.kBrushless);
  private final CANSparkMax bottomSpinMotor =
      new CANSparkMax(IdConstants.CANId.BottomSpinMotorId, CANSparkLowLevel.MotorType.kBrushless);
  private final CANSparkMax articulationMotor =
      new CANSparkMax(3, CANSparkLowLevel.MotorType.kBrushed);

  private final RelativeEncoder topEncoder;
  private final RelativeEncoder bottomEncoder;

  private final RelativeEncoder articulationEncoder;

  public ShooterIOReal() {
    topEncoder = topSpinMotor.getEncoder();
    bottomEncoder = bottomSpinMotor.getEncoder();
    topSpinMotor.setInverted(true);
    articulationEncoder = articulationMotor.getEncoder();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.bottomPositionRad = Units.rotationsToRadians(bottomEncoder.getPosition());
    inputs.topPositionRad = Units.rotationsToRadians(topEncoder.getPosition());
    inputs.articulationPosition = Rotation2d.fromRotations(topEncoder.getPosition());

    inputs.bottomVelocityRadPerSec = Units.rotationsToRadians(bottomEncoder.getVelocity());
    inputs.topVelocityRadPerSec = Units.rotationsToRadians(topEncoder.getVelocity());
    inputs.articulationVelocityRadPerSec =
        Units.rotationsToRadians(articulationEncoder.getVelocity());

    inputs.bottomAppliedVolts =
        bottomSpinMotor.getAppliedOutput() * bottomSpinMotor.getBusVoltage();
    inputs.topAppliedVolts = topSpinMotor.getAppliedOutput() * topSpinMotor.getBusVoltage();
    inputs.articulationAppliedVolts =
        articulationMotor.getAppliedOutput() * articulationMotor.getBusVoltage();
  }

  @Override
  public void setArticulationVoltage(double volts) {
    articulationMotor.setVoltage(volts);
  }

  @Override
  public void setTopVoltage(double volts) {
    topSpinMotor.setVoltage(volts);
  }

  @Override
  public void setBottomVoltage(double volts) {
    bottomSpinMotor.setVoltage(volts);
  }
}
