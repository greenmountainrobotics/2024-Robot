package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.DriveConstants;
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

    // controller1.y().onTrue(shooter.prepareToShoot(drive::getPose));
    // controller1.a().onTrue(shooter.runAtRPM(5000));
    // controller1.b().onTrue(shooter.stop());
    /*controller1.a().whileTrue(drive.runToPose(FieldPoseUtils::alignedWithAmpPose));
    controller1.b().whileTrue(drive.runToPose(FieldPoseUtils::alignedWithSourcePose));*/
    controller1
        .a()
        .onTrue(new InstantCommand(() -> lockedToSpeaker = true))
        .onFalse(new InstantCommand(() -> lockedToSpeaker = false));

    controller1
        .x()
        .onTrue(intake.shoot(-1).andThen(intake.extend()))
        .onFalse(intake.shoot(0).andThen(intake.retract()));

    controller1
        .y()
        .onTrue(
            shooter
                .runAtRPM(5000)
                .andThen(intake.shoot(1))
                .andThen(new WaitCommand(1))
                .andThen(intake.shoot(0))
                .andThen(shooter.runAtRPM(0).alongWith(intake.shoot(0))))
        .onFalse(shooter.runAtRPM(0).alongWith(intake.shoot(0)));

    controller1
        .b()
        .whileTrue(
            drive.runToPose(
                () ->
                    FieldPoseUtils.flipPoseIfRed(
                        new Pose2d(
                            FieldConstants.SpeakerCloseSideCenter.getX()
                                + DriveConstants.WidthWithBumpersX / 2,
                            FieldConstants.SpeakerCloseSideCenter.getY(),
                            new Rotation2d()))));

    controller1.povLeft().onTrue(new InstantCommand(() -> drive.setPose(new Pose2d())));

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
                        FieldPoseUtils.flipTranslationIfRed(FieldConstants.SpeakerCloseSideCenter)
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
