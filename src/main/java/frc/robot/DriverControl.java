package frc.robot;

import static frc.robot.Commands.*;
import static frc.robot.Config.SINGLE_CONTROLLER;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.ShooterConstants;
import frc.robot.util.Alliance;

public class DriverControl {
  private final CommandXboxController controller1 = new CommandXboxController(0);
  private final CommandXboxController controller2 = new CommandXboxController(1);

  public DriverControl(Robot robot) {
    var driveController = SINGLE_CONTROLLER ? controller2 : controller1;
    var drive = robot.drive;
    var intake = robot.intake;
    var shooter = robot.shooter;

    // intake
    controller2
        .leftTrigger()
        .onTrue(intake.setShooter(-1).until(intake::isLimitSwitchPressed).andThen(intake.extend()))
        .onFalse(intake.setShooter(0).andThen(intake.retract()));

    // shoot
    controller2
        .rightTrigger()
        .whileTrue(
            shootInSpeaker(shooter, drive, intake)
                .alongWith(
                    new InstantCommand(
                        () ->
                            driveController
                                .getHID()
                                .setRumble(GenericHID.RumbleType.kBothRumble, 1))))
        .onFalse(
            stopShooting(shooter, intake)
                .alongWith(
                    new InstantCommand(
                        () ->
                            driveController
                                .getHID()
                                .setRumble(GenericHID.RumbleType.kBothRumble, 0))));

    controller2.a().whileTrue(intake.shoot(-1));

    // apriltags on and off
    controller2
        .povLeft()
        .and(controller2.a())
        .onTrue(
            new InstantCommand(
                () ->
                    robot.aprilTagVision.setDataInterface(
                        drive::addVisionMeasurement, drive::getPose)));

    controller2
        .povLeft()
        .and(controller2.b())
        .onTrue(
            new InstantCommand(
                () -> {
                  robot.aprilTagVision.setDataInterface((pose2d, aDouble) -> {}, Pose2d::new);
                  drive.setPose(new Pose2d());
                }));

    // manually control intake / shooter
    controller2.povRight().whileTrue(intake.shoot(() -> -controller2.getRightY()));

    controller2
        .povDown()
        .and(controller2.a())
        .onTrue(shooter.runAtRPM(ShooterConstants.ShootingVelocityRPM));

    controller2.povDown().and(controller2.b()).onTrue(shooter.runAtRPM(0));

    controller2
        .b()
        .whileTrue(shootInSpeaker(shooter,drive,intake, false))
        .onFalse(stopShooting(shooter, intake));
    controller2
        .y()
        .whileTrue(shootInAmp(shooter, drive, intake, true))
        .onFalse(stopShooting(shooter, intake));

    driveController.x().onTrue(new InstantCommand(drive::stopWithX, drive));

    drive.setDefaultCommand(
        new RunCommand(
            () -> {
              var DEADBAND = 0.25;

              var x = -driveController.getLeftY();
              var y = -driveController.getLeftX();
              var omega = 0.0;

              omega = -driveController.getRightX();
              omega = MathUtil.applyDeadband(omega, DEADBAND);
              omega = Math.copySign(omega * omega, omega);
              omega = omega * drive.getMaxAngularSpeedRadPerSec();

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
