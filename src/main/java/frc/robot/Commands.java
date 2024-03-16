package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.FieldPoseUtils;

public class Commands {
  public static Command shootInSpeaker(Shooter shooter, Drive drive, Intake intake, boolean align) {
    return shooter
        .runAtRPM(5000)
        .alongWith(
            new ConditionalCommand(
                drive.alignToSpeaker(), new InstantCommand(() -> {}), () -> align))
        .alongWith(intake.setShooter(0).andThen(intake.retract()))
        .andThen(intake.shoot(1).withTimeout(0.5))
        .andThen(shooter.runAtRPM(0));
  }

  public static Command shootInSpeaker(Shooter shooter, Drive drive, Intake intake) {
    return shootInSpeaker(shooter, drive, intake, true);
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
                .andThen(intake.setShooter(-1))
                .andThen(intake.extend()));
  }
}
