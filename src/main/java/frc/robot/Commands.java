package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.FieldPoseUtils;

public class Commands {
  public static Command shootInSpeaker(
      Shooter shooter, Drive drive, Intake intake, boolean align, double rpm) {
    return new ConditionalCommand(
            new WaitUntilCommand(
                () ->
                    drive.distanceFromPoint(
                            FieldPoseUtils.flipTranslationIfRed(
                                FieldConstants.SpeakerCloseSideCenter))
                        < FieldConstants.SpeakerShootingDistance + 0.2),
            new InstantCommand(() -> {}),
            () -> align)
        .andThen(shooter.setRPM(rpm * 5 / 6))
        .alongWith(
            new ConditionalCommand(
                drive.alignToSpeaker(), new InstantCommand(() -> {}), () -> align))
        .andThen(shooter.runAtRPM(rpm))
        .andThen(intake.shoot(1).withTimeout(0.5))
        .andThen(shooter.runAtRPM(0));
  }

  public static Command shootInSpeaker(Shooter shooter, Drive drive, Intake intake, boolean align) {
    return shootInSpeaker(shooter, drive, intake, align, ShooterConstants.ShootingVelocityRPM);
  }

  public static Command shootInSpeaker(Shooter shooter, Drive drive, Intake intake) {
    return shootInSpeaker(shooter, drive, intake, true);
  }

  public static Command shootInAmp(Shooter shooter, Drive drive, Intake intake, boolean align) {
    return new ConditionalCommand(
            drive.alignToFrontOfAmp(), new InstantCommand(() -> {}), () -> align)
        .alongWith(intake.setShooter(0).andThen(intake.retract()))
        .andThen(
            shooter
                .runAtRPM(
                    () -> -SmartDashboard.getNumber("amp speed", 900),
                    () ->
                        SmartDashboard.getNumber("amp speed", 900)
                            * SmartDashboard.getNumber("amp ratio", 4.5))
                .andThen(
                    intake
                        .shoot(1)
                        .withTimeout(0.5)
                        .alongWith(
                            new ConditionalCommand(
                                drive.alignToAmp(), new InstantCommand(() -> {}), () -> align))))
        .andThen(shooter.runAtRPM(0));
  }

  public static Command shootInAmp(Shooter shooter, Drive drive, Intake intake) {
    return shootInAmp(shooter, drive, intake, true);
  }

  public static Command stopShooting(Shooter shooter, Intake intake) {
    return shooter.runAtRPM(0).alongWith(intake.setShooter(0));
  }

  public static Command intakeFromGround(Translation2d notePosition, Intake intake, Drive drive) {
    Translation2d finalNotePosition = FieldPoseUtils.flipTranslation(notePosition);
    return drive
        .alignToNote(finalNotePosition)
        .alongWith(
            new WaitUntilCommand(
                    () ->
                        drive.distanceFromPoint(finalNotePosition)
                            < DriveConstants.WidthWithBumpersX * 2)
                .andThen(intake.setShooter(-0.5))
                .andThen(intake.extend()))
        .alongWith(new WaitCommand(1));
  }
}
