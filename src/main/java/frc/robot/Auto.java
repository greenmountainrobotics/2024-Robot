package frc.robot;

import static frc.robot.Commands.intakeFromGround;
import static frc.robot.Commands.shootInSpeaker;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.Trajectory;
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class Auto {
  private final LoggedDashboardNumber autoDelay;

  private final LoggedDashboardBoolean shootTop;
  private final LoggedDashboardBoolean shootMiddle;
  private final LoggedDashboardBoolean shootBottom;

  private final LoggedDashboardBoolean knockOut;
  private final LoggedDashboardBoolean taxi;


  private final LoggedDashboardChooser<String> preloadedNoteShoot;

  private Command currentCommand;

  private final Robot robot;

  public Auto(Robot robot) {
    this.robot = robot;

    autoDelay = new LoggedDashboardNumber("Auto Delay", 0);

    shootMiddle = new LoggedDashboardBoolean("Middle -> Shoot", false);
    shootBottom = new LoggedDashboardBoolean("Bottom -> Shoot", false);
    shootTop = new LoggedDashboardBoolean("Top -> Shoot", false);

    preloadedNoteShoot = new LoggedDashboardChooser<>("Preloaded Note Action");
    preloadedNoteShoot.addDefaultOption("None", "None");
    preloadedNoteShoot.addOption("Beginning", "Beginning");
    preloadedNoteShoot.addOption("End", "End");

    knockOut = new LoggedDashboardBoolean("Gremlin Path", false);
    taxi = new LoggedDashboardBoolean("Taxi", false);
  }

  public void schedule() {
    currentCommand.schedule();
  }

  public void periodic() {
    var drive = robot.drive;
    var shooter = robot.shooter;
    var intake = robot.intake;

    currentCommand = new InstantCommand(() -> {});

    if (autoDelay.get() != 0)
      currentCommand = currentCommand.andThen(new WaitCommand(autoDelay.get()));

    if (preloadedNoteShoot.get().equals("Beginning"))
      currentCommand = currentCommand.andThen(shootInSpeaker(shooter, drive, intake));

    if (knockOut.get())
      currentCommand = currentCommand.andThen(drive.followPath(Trajectory.KnockOutMiddle));

    if (taxi.get())
      currentCommand = currentCommand.andThen(drive.followPath(Trajectory.Taxi));

    if (shootMiddle.get())
      currentCommand =
          currentCommand
              .andThen(intakeFromGround(FieldConstants.MiddleInnerNote, intake, drive))
              .andThen(shootInSpeaker(shooter, drive, intake));

    if (shootBottom.get())
      currentCommand =
          currentCommand
              .andThen(intakeFromGround(FieldConstants.BottomInnerNote, intake, drive))
              .andThen(shootInSpeaker(shooter, drive, intake));

    if (shootTop.get())
      currentCommand =
          currentCommand
              .andThen(intakeFromGround(FieldConstants.TopInnerNote, intake, drive))
              .andThen(shootInSpeaker(shooter, drive, intake));

    if (preloadedNoteShoot.get().equals("End"))
      currentCommand = currentCommand.andThen(shootInSpeaker(shooter, drive, intake));
  }

  public void cancel() {
    if (currentCommand != null) currentCommand.cancel();
  }
}
