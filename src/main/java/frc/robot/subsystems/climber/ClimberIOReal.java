package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import frc.robot.constants.IdConstants;

public class ClimberIOReal implements ClimberIO {
  private final VictorSP leftMotor = new VictorSP(IdConstants.PWMId.LeftClimberId);
  private final VictorSP rightMotor = new VictorSP(IdConstants.PWMId.RightClimberId);

  public ClimberIOReal() {}

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.leftOutput = leftMotor.get();
    inputs.rightOutput = rightMotor.get();
  }

  @Override
  public void setLeftVoltage(double volts) {
    leftMotor.setVoltage(volts);
  }

  @Override
  public void setRightVoltage(double volts) {
    rightMotor.set(volts);
  }
}
