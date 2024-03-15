package frc.robot;

import static frc.robot.Commands.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.FieldConstants;
import frc.robot.util.Alliance;

public class DriverControl {
  private final CommandXboxController controller1 = new CommandXboxController(0);
  private final CommandXboxController controller2 = new CommandXboxController(1);

  public DriverControl(Robot robot) {
    var drive = robot.drive;
    var intake = robot.intake;
    var shooter = robot.shooter;

    // intake
    controller1
        .x()
        .onTrue(intake.setShooter(-1).andThen(intake.extend()))
        .onFalse(intake.setShooter(0).andThen(intake.retract()));

    // shoot
    controller1
        .y()
        .whileTrue(shootInSpeaker(shooter, drive, intake))
        .onFalse(stopShooting(shooter, intake));

    controller1
        .a()
        .whileTrue(
            shootInSpeaker(shooter, drive, intake)
                .andThen(intakeFromGround(FieldConstants.BottomInnerNote, intake, drive))
                .andThen(shootInSpeaker(shooter, drive, intake))
                .andThen(intakeFromGround(FieldConstants.MiddleInnerNote, intake, drive))
                .andThen(shootInSpeaker(shooter, drive, intake))
                .andThen(intakeFromGround(FieldConstants.TopInnerNote, intake, drive))
                .andThen(shootInSpeaker(shooter, drive, intake)));

    drive.setDefaultCommand(
        new RunCommand(
            () -> {
              var DEADBAND = 0.25;

              var x = -controller1.getLeftY();
              var y = -controller1.getLeftX();
              var omega = 0.0;

              omega = -controller1.getRightX();
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
