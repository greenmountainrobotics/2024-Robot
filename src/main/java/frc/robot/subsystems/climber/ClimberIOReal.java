package frc.robot.subsystems.climber;

import static edu.wpi.first.math.MathUtil.angleModulus;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import frc.robot.constants.IdConstants;
import frc.robot.constants.ShooterConstants;

public class ClimberIOReal implements ClimberIO {
    private final VictorSP leftMotor = new VictorSP(IdConstants.PWMId.LeftClimberId);
    private final VictorSP rightMotor = new VictorSP(IdConstants.PWMId.RightClimberId);

    public ClimberIOReal() {
    }

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
