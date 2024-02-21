package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.constants.TunableConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.util.RunMode;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private final PIDController topPID;
  private final PIDController bottomPID;

  private final PIDController articulationPID;

  private final SimpleMotorFeedforward topFF;
  private final SimpleMotorFeedforward bottomFF;
  private final SysIdRoutine flywheelSysId;

  private double topSetpointRPM = 0.0;
  private double bottomSetpointRPM = 0.0;
  private Rotation2d articulationSetpoint = Rotation2d.fromRadians(Math.PI);

  public Shooter(ShooterIO io) {
    this.io = io;

    switch (RunMode.getMode()) {
      case REAL:
      case REPLAY:
        topFF = new SimpleMotorFeedforward(KsTopFlywheel, KvTopFlywheel);
        bottomFF = new SimpleMotorFeedforward(KsBottomFlywheel, KvBottomFlywheel);
        topPID = new PIDController(KpTopFlywheel, 0, KdTopFlywheel);
        bottomPID = new PIDController(KpBottomFlywheel, 0, KdBottomFlywheel);
        articulationPID = new PIDController(KpShooterArticulation, 0, KdShooterArticulation);
        break;
      default:
        // simulated
        topFF = new SimpleMotorFeedforward(0, .005);
        bottomFF = new SimpleMotorFeedforward(0, .005);
        topPID = new PIDController(1, 0, 0);
        bottomPID = new PIDController(1, 0, 0);
        articulationPID = new PIDController(0.1, 0, 0);
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

    io.setTopVoltage(
        topPID.calculate(
                Units.radiansPerSecondToRotationsPerMinute(inputs.topVelocityRadPerSec),
                topSetpointRPM)
            + topFF.calculate(topSetpointRPM));

    io.setBottomVoltage(
        bottomPID.calculate(
                Units.radiansPerSecondToRotationsPerMinute(inputs.bottomVelocityRadPerSec),
                bottomSetpointRPM)
            + bottomFF.calculate(bottomSetpointRPM));

    io.setArticulationVoltage(
        articulationPID.calculate(
            inputs.articulationPosition.getRadians(), articulationSetpoint.getRadians()));

    Logger.recordOutput("Shooter/TopSetpointRPM", topSetpointRPM);
    Logger.recordOutput(
        "Shooter/TopRPM", Units.radiansPerSecondToRotationsPerMinute(inputs.topVelocityRadPerSec));

    Logger.recordOutput("Shooter/BottomSetpointRPM", bottomSetpointRPM);
    Logger.recordOutput(
        "Shooter/BottomRPM",
        Units.radiansPerSecondToRotationsPerMinute(inputs.bottomVelocityRadPerSec));

    Logger.recordOutput("Shooter/ArticulationSetpoint", articulationSetpoint);
    Logger.recordOutput("Shooter/Articulation", inputs.articulationPosition);

    Logger.recordOutput("Shooter/RealMechanism", getMechanism(inputs.articulationPosition));
    Logger.recordOutput("Shooter/TargetMechanism", getMechanism(articulationSetpoint));
  }

  private Mechanism2d getMechanism(Rotation2d articulation) {
    Mechanism2d mechanism = new Mechanism2d(DriveConstants.WidthWithBumpersX, 1);
    MechanismRoot2d mechRoot =
        mechanism.getRoot(
            "shooter",
            DriveConstants.WidthWithBumpersX / 2 + ShooterConstants.PivotX,
            ShooterConstants.PivotHeight);
    MechanismLigament2d elevator =
        mechRoot.append(new MechanismLigament2d("shooter pivot", 0.3, articulation.getDegrees()));
    return mechanism;
  }

  public void setFlywheelSetpointRPM(double top, double bottom) {
    topSetpointRPM = top;
    bottomSetpointRPM = bottom;
  }

  public void setArticulation(Rotation2d articulation) {
    articulationSetpoint = articulation;
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
