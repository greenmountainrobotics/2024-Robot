package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class Auto {
  private final LoggedDashboardChooser<Command> autoChooser;
  private final LoggedDashboardNumber autoDelay;

  private Command currentCommand;

  public Auto(Robot robot) {
    var drive = robot.drive;
    var shooter = robot.shooter;

    autoChooser = new LoggedDashboardChooser<>("Auto Choices");
    autoDelay = new LoggedDashboardNumber("Auto Delay", 0);

    // No-op
    autoChooser.addDefaultOption("None", new InstantCommand(() -> {}));
  }

  public void schedule() {
    if (autoDelay.get() != 0) {
      currentCommand =
          new SequentialCommandGroup(new WaitCommand(autoDelay.get()), autoChooser.get());
    } else if (autoChooser.get() == null) {
      currentCommand = new InstantCommand(() -> {});
    } else {
      currentCommand = autoChooser.get();
    }
    currentCommand.schedule();
  }

  public void cancel() {
    if (currentCommand != null) currentCommand.cancel();
  }
}
