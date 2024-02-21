package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

  private double extensionSetpointMeters = 0.0;
  private Rotation2d articulationSetpoint = Rotation2d.fromDegrees(90);

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
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    io.articulationRunVoltage(articulationPID.calculate(inputs.articulationPosition.getRadians()));

    double currentExtensionMeters =
        IntakeConstants.ExtensionMetersPerRotation
                * Rotation2d.fromRadians(inputs.leftExtensionPositionRad).getRotations()
            + IntakeConstants.MinExtension;

    var voltage = extensionPID.calculate(currentExtensionMeters - IntakeConstants.MinExtension);
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
                "wrist", 0.1, articulation.getDegrees(), 10, new Color8Bit(Color.kGreen)));
    return mechanism;
  }

  public void setIntakeSpeed(double speed) {
    io.spinRunVoltage(12.0 * speed);
  }

  public void setArticulation(Rotation2d rotation) {
    articulationPID.setSetpoint(rotation.getRadians());
    articulationSetpoint = rotation;
  }

  public void setExtension(double coeff) {
    extensionSetpointMeters =
        coeff * (IntakeConstants.MaxExtension - IntakeConstants.MinExtension)
            + IntakeConstants.MinExtension;
    extensionPID.setSetpoint(extensionSetpointMeters - IntakeConstants.MinExtension);
  }
}
