package frc.robot.subsystems.intake;

import static edu.wpi.first.math.MathUtil.angleModulus;
import static frc.robot.constants.IntakeConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.TunableConstants;
import frc.robot.util.RunMode;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final IntakeIO io;

  private final PIDController extensionPID;
  private final PIDController articulationPID;

  private double extensionSetpointMeters = MinExtension;
  private double currentExtensionMeters = 0.0;
  private Rotation2d articulationSetpoint = RetractedArticulation;

  public Intake(IntakeIO io) {
    this.io = io;

    switch (RunMode.getMode()) {
      case REAL, REPLAY -> {
        extensionPID =
            new PIDController(
                TunableConstants.KpIntakeExtension, 0, TunableConstants.KdIntakeExtension);
        articulationPID =
            new PIDController(
                TunableConstants.KpIntakeArticulation, 0, TunableConstants.KdIntakeArticulation);
      }
      default -> {
        extensionPID = new PIDController(15, 0, 3);
        articulationPID = new PIDController(5, 0, 0.8);
      }
    }

    extensionPID.setSetpoint(extensionSetpointMeters);
    articulationPID.setSetpoint(angleModulus(articulationSetpoint.getRadians()));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    io.articulationRunVoltage(articulationPID.calculate(inputs.articulationPosition.getRadians()));

    currentExtensionMeters =
        IntakeConstants.ExtensionMetersPerRotation
                * Rotation2d.fromRadians(inputs.leftExtensionPositionRad).getRotations()
            + IntakeConstants.MinExtension;

    var voltage = extensionPID.calculate(currentExtensionMeters);
    io.extensionRunVoltage(voltage, voltage);

    Logger.recordOutput(
        "Intake/RealMechanism", getMechanism(currentExtensionMeters, inputs.articulationPosition));
    Logger.recordOutput(
        "Intake/TargetMechanism", getMechanism(extensionSetpointMeters, articulationSetpoint));
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

  public void setIntakeSpeed(double speed) {
    io.spinRunVoltage(12.0 * speed);
  }

  public void setArticulation(Rotation2d rotation) {
    articulationPID.setSetpoint(angleModulus(rotation.getRadians()));
    articulationSetpoint = rotation;
  }

  public void setExtension(double extensionSetpointMeters) {
    this.extensionSetpointMeters = extensionSetpointMeters;
    extensionPID.setSetpoint(extensionSetpointMeters);
  }

  public boolean extensionIsAtSetpoint() {
    return Math.abs(extensionSetpointMeters - currentExtensionMeters) < ExtensionTolerance;
  }

  public boolean articulationIsAtSetpoint() {
    return Math.abs(extensionSetpointMeters - currentExtensionMeters) < ArticulationToleranceRad;
  }

  public double getExtension() {
    return currentExtensionMeters;
  }

  public Rotation2d getArticulation() {
    return inputs.articulationPosition;
  }

  public Command extend() {
    return new InstantCommand(() -> setExtension(TargetExtension), this)
        .andThen(new InstantCommand(() -> setArticulation(HalfwayArticulation), this))
        .andThen(
            new RunCommand(
                () -> {
                  if (currentExtensionMeters > StartRotatingDownwardsExtension)
                    setArticulation(
                        HalfwayArticulation.plus(
                            TargetArticulation.minus(HalfwayArticulation)
                                .times(
                                    (currentExtensionMeters - StartRotatingDownwardsExtension)
                                        / (TargetExtension - StartRotatingDownwardsExtension))));
                },
                this))
        .until(() -> articulationIsAtSetpoint() && extensionIsAtSetpoint());
  }

  public Command retract() {
    return new InstantCommand(() -> setExtension(MinExtension), this)
        .andThen(new InstantCommand(() -> setArticulation(RetractedArticulation), this))
        .andThen(new WaitUntilCommand(() -> articulationIsAtSetpoint() && extensionIsAtSetpoint()));
  }
}
