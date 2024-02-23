package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.FieldConstants;
import frc.robot.util.Alliance;
import frc.robot.util.FieldPoseUtils;

public class DriverControl {
  private final CommandXboxController controller1 = new CommandXboxController(0);
  private final CommandXboxController controller2 = new CommandXboxController(1);
  private boolean lockedToSpeaker = false;

  public DriverControl(Robot robot) {
    var drive = robot.drive;
    var intake = robot.intake;
    var shooter = robot.shooter;

    controller1.y().onTrue(shooter.prepareToShoot(drive::getPose));
    controller1.a().whileTrue(drive.runToPose(FieldPoseUtils::alignedWithAmpPose));
    controller1.b().whileTrue(drive.runToPose(FieldPoseUtils::alignedWithSourcePose));
    controller1.x().onTrue(new InstantCommand(() -> lockedToSpeaker = !lockedToSpeaker));

    drive.setDefaultCommand(
        new RunCommand(
            () -> {
              var DEADBAND = 0.25;

              var x = -controller1.getLeftY();
              var y = -controller1.getLeftX();
              var omega = 0.0;

              if (lockedToSpeaker) {
                omega =
                    drive.calculatePIDThetaVelocity(
                        FieldPoseUtils.flipTranslationIfRed(FieldConstants.SpeakerFarSideCenter)
                            .minus(drive.getPose().getTranslation())
                            .getAngle()
                            .minus(Rotation2d.fromRadians(Math.PI))
                            .getRadians(),
                        drive.getPose().getRotation().getRadians());
              } else {
                omega = -controller1.getRightX();
                omega = MathUtil.applyDeadband(omega, DEADBAND);
                omega = Math.copySign(omega * omega, omega);
                omega = omega * drive.getMaxAngularSpeedRadPerSec();
              }

              double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
              Rotation2d linearDirection = new Rotation2d(x, y);

              linearMagnitude = linearMagnitude * linearMagnitude;

              // Calcaulate new linear velocity
              Translation2d linearVelocity =
                  new Pose2d(new Translation2d(), linearDirection)
                      .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                      .getTranslation();

              // Convert to field relative speeds & send command
              boolean isFlipped = Alliance.isRed();
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));
            },
            drive));
  }
}
