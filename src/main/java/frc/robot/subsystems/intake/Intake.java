package frc.robot.subsystems.intake;

import static edu.wpi.first.math.MathUtil.angleModulus;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.constants.IntakeConstants.*;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.TunableConstants;
import frc.robot.util.RunMode;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final IntakeIO io;

  private final PIDController extensionPID;
  private final ProfiledPIDController articulationPID;

  private final ArmFeedforward articulationFF;

  private double extensionSetpointMeters = MinExtension;
  private double currentExtensionMeters = 0.0;
  private Rotation2d articulationSetpoint = RetractedArticulation;

  private boolean extended;

  private SysIdRoutineLog.State sysIdState = SysIdRoutineLog.State.kNone;
  private final SysIdRoutine articulationSysId;

  private double prevArticulation = 0.0;
  private double prevTimestamp = 0.0;

  public Intake(IntakeIO io) {
    this.io = io;

    switch (RunMode.getMode()) {
      case REAL, REPLAY -> {
        extensionPID =
            new PIDController(
                TunableConstants.KpIntakeExtension, 0, TunableConstants.KdIntakeExtension);
        articulationPID =
            new ProfiledPIDController(
                TunableConstants.KpIntakeArticulation,
                0,
                TunableConstants.KdIntakeArticulation,
                new TrapezoidProfile.Constraints(200, 100));
        articulationFF =
            new ArmFeedforward(
                TunableConstants.KsIntakeArticulation,
                TunableConstants.KgIntakeArticulation,
                TunableConstants.KvIntakeArticulation,
                TunableConstants.KaIntakeArticulation);
      }
      default -> {
        extensionPID = new PIDController(25, 0, 4);
        articulationPID =
            new ProfiledPIDController(1, 0, 0.2, new TrapezoidProfile.Constraints(1, 1));
        articulationFF = new ArmFeedforward(0, 0, 0, 0);
      }
    }

    extensionPID.setSetpoint(extensionSetpointMeters);
    articulationPID.setGoal(angleModulus(articulationSetpoint.getRadians()));

    articulationSysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(null, null, null, state -> sysIdState = state),
            new SysIdRoutine.Mechanism(
                voltageMeasure -> {
                  io.articulationRunVoltage(voltageMeasure.in(Volts));
                },
                null,
                this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    Logger.recordOutput(
        "Intake/RealMechanism", getMechanism(currentExtensionMeters, inputs.articulationPosition));
    Logger.recordOutput(
        "Intake/TargetMechanism", getMechanism(extensionSetpointMeters, articulationSetpoint));
    Logger.recordOutput("Intake/SysIdState", sysIdState.toString());
    Logger.recordOutput("Intake/ArticulationPositionRad", inputs.articulationPosition.getRadians());
    Logger.recordOutput(
        "Intake/ArticulatinonVelocity",
        (inputs.articulationPosition.getRadians() - prevArticulation)
            / (Timer.getFPGATimestamp() - prevTimestamp));
    prevArticulation = inputs.articulationPosition.getRadians();
    prevTimestamp = Timer.getFPGATimestamp();

    if (sysIdState != SysIdRoutineLog.State.kNone) return;

    io.articulationRunVoltage(
        articulationFF.calculate(inputs.articulationPosition.getRadians(), 0, 0)
            + articulationPID.calculate(inputs.articulationPosition.getRadians()));

    currentExtensionMeters =
        IntakeConstants.ExtensionMetersPerRotation
                * Rotation2d.fromRadians(inputs.leftExtensionPositionRad).getRotations()
            + IntakeConstants.StartExtension;

    var voltage = extensionPID.calculate(currentExtensionMeters);
    voltage = Math.abs(voltage) > 0.2 ? voltage : 0;
    io.extensionRunVoltage(voltage);

    SmartDashboard.putBoolean("INTAKE", noteIsIntaked());
  }

  private Mechanism2d getMechanism(double extension, Rotation2d articulation) {
    Mechanism2d mechanism = new Mechanism2d(DriveConstants.WidthWithBumpersX, 1);
    MechanismRoot2d mechRoot =
        mechanism.getRoot(
            "intake",
            DriveConstants.WidthWithBumpersX / 2 + IntakeConstants.ExtensionStartX,
            IntakeConstants.PivotHeight);
    MechanismLigament2d elevator =
        mechRoot.append(new MechanismLigament2d("intake extension", extension, 0));
    MechanismLigament2d wrist =
        elevator.append(
            new MechanismLigament2d(
                "wrist", 0.1, articulation.getDegrees(), 5, new Color8Bit(Color.kGreen)));
    return mechanism;
  }

  /**
   * @param speed positive is outwards
   */
  public void setIntakeSpeed(double speed) {
    io.spinRunVoltage(12.0 * speed);
  }

  public void setArticulation(Rotation2d rotation) {
    articulationPID.setGoal(angleModulus(rotation.getRadians()));
    articulationSetpoint = rotation;
  }

  public void setExtension(double extensionSetpointMeters) {
    this.extensionSetpointMeters = extensionSetpointMeters;
    extensionPID.setSetpoint(extensionSetpointMeters);
  }

  @AutoLogOutput
  public boolean extensionIsAtSetpoint() {
    return Math.abs(extensionSetpointMeters - currentExtensionMeters) < ExtensionTolerance;
  }

  @AutoLogOutput
  public boolean articulationIsAtSetpoint() {
    return Math.abs(getArticulation().minus(articulationSetpoint).getRadians())
        < ArticulationToleranceRad;
  }

  public double getExtension() {
    return currentExtensionMeters;
  }

  public Rotation2d getArticulation() {
    return inputs.articulationPosition;
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command articulationSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return articulationSysId.quasistatic(direction);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command articulationSysIdDynamic(SysIdRoutine.Direction direction) {
    return articulationSysId.dynamic(direction);
  }

  public Command extend() {
    return new InstantCommand(
            () -> {
              setExtension(TargetExtension);
              setArticulation(HalfwayArticulation);
              extended = true;
            },
            this)
        .andThen(
            new RunCommand(
                    () -> {
                      if (currentExtensionMeters > StartRotatingDownwardsExtension)
                        setArticulation(
                            HalfwayArticulation.plus(
                                TargetArticulation.minus(HalfwayArticulation)
                                    .times(
                                        (currentExtensionMeters - StartRotatingDownwardsExtension)
                                            / (TargetExtension
                                                - StartRotatingDownwardsExtension))));
                    },
                    this)
                .until(() -> articulationIsAtSetpoint() && extensionIsAtSetpoint()));
  }

  public Command retract() {
    return retract(RetractedArticulation);
  }

  public Command retract(Rotation2d articulation) {
    return new InstantCommand(
            () -> {
              setArticulation(articulation);
              extended = false;
            },
            this)
        .andThen(
            new WaitUntilCommand(
                () ->
                    currentExtensionMeters < StartRotatingDownwardsExtension
                        || getArticulation().minus(HalfwayArticulation).getRadians() > 0))
        .andThen(new InstantCommand(() -> setExtension(MinExtension), this))
        .andThen(new WaitUntilCommand(() -> articulationIsAtSetpoint() && extensionIsAtSetpoint()));
  }

  public Command toggleExtension() {
    return new ConditionalCommand(retract(), extend(), () -> extended);
  }

  /**
   * @param speed positive is outwards
   */
  public Command shoot(double speed) {
    return shoot(() -> speed);
  }

  public Command shoot(Supplier<Double> speedSupplier) {
    return new RunCommand(() -> setIntakeSpeed(speedSupplier.get()), this)
        .finallyDo(() -> setIntakeSpeed(0));
  }

  /**
   * @param speed positive is outwards
   */
  public Command setShooter(double speed) {
    return new InstantCommand(() -> setIntakeSpeed(speed));
  }

  @AutoLogOutput
  public boolean isLimitSwitchPressed() {
    return inputs.limitSwitchPressed;
  }

  @AutoLogOutput
  public boolean noteIsIntaked() {
    return inputs.spinCurrentAmps > 5;
  }
}
