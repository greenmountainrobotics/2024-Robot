package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConstants;
import frc.robot.util.RunMode;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final IntakeIO io;

  private final PIDController extensionPID;
  private final PIDController articulationPID;

  private double extensionSetpoint = 0.0;
  private Rotation2d articulationSetpoint = Rotation2d.fromDegrees(90);

  public Intake(IntakeIO io) {
    this.io = io;

    switch (RunMode.getMode()) {
      case REAL, REPLAY -> {
        extensionPID = new PIDController(1.1, 0, 0);
        articulationPID = new PIDController(1.1, 0, 0);
      }
      default -> {
        extensionPID = new PIDController(1, 0, 0);
        articulationPID = new PIDController(1, 0, 0);
      }
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    io.articulationRunVoltage(articulationPID.calculate(inputs.articulationPosition.getRadians()));
    io.extensionRunVoltage(
        extensionPID.calculate(inputs.leftExtensionPositionRad),
        extensionPID.calculate(inputs.leftExtensionPositionRad));

    //Logger.recordOutput("Intake/RealMechanism", getMechanism(inputs.leftExtensionPositionRad));
    Logger.recordOutput(
        "Intake/TargetMechanism", getMechanism(extensionSetpoint, articulationSetpoint));
  }

  private Mechanism2d getMechanism(double extension, Rotation2d articulation) {
    // TODO: THIS DOES NOT WORK
    Mechanism2d mechanism = new Mechanism2d(RobotConstants.WidthWithBumpersX, 1);
    MechanismRoot2d mechRoot = mechanism.getRoot("intake", 0, 0);
    MechanismLigament2d elevator =
        mechRoot.append(new MechanismLigament2d("intake extension", 1, 90));
    MechanismLigament2d wrist =
        elevator.append(new MechanismLigament2d("wrist", 0.5, articulation.getDegrees()));
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
    extensionSetpoint = coeff;
  }
}
