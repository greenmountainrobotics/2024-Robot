package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private ShooterIO io;
  private ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private PIDController topPID;
  private PIDController bottomPID;

  private SimpleMotorFeedforward topFF;
  private SimpleMotorFeedforward bottomFF;
  private SysIdRoutine flywheelSysId;

  public Shooter(ShooterIO io) {
    this.io = io;

    switch (Constants.getMode()) {
      case REAL:
      case REPLAY:
        topFF = new SimpleMotorFeedforward(0, 0);
        bottomFF = new SimpleMotorFeedforward(0, 0);
        topPID = new PIDController(1, 0, 0);
        bottomPID = new PIDController(1, 0, 0);
        break;
      case SIM:
        topFF = new SimpleMotorFeedforward(0, 0);
        bottomFF = new SimpleMotorFeedforward(0, 0);
        topPID = new PIDController(1, 0, 0);
        bottomPID = new PIDController(1, 0, 0);
        break;
    }

    flywheelSysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                state -> Logger.recordOutput("Shooter/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                voltageMeasure -> {
                  io.setTopVoltage(voltageMeasure.in(Volts));
                  io.setBottomVoltage(voltageMeasure.in(Volts));
                },
                null,
                this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }

  public void setFlywheelSetpointRPM(double top, double bottom) {
    io.setTopVoltage(
        topPID.calculate(
                Units.radiansPerSecondToRotationsPerMinute(inputs.topVelocityRadPerSec), top)
            + topFF.calculate(top));
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command flywheelSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return flywheelSysId.quasistatic(direction);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command flywheelSysIdDynamic(SysIdRoutine.Direction direction) {
    return flywheelSysId.dynamic(direction);
  }
}
