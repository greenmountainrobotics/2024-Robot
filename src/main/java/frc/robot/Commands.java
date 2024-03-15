package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class Commands {
    public static Command shootInSpeaker(Shooter shooter, Drive drive, Intake intake) {
        return shooter
                .runAtRPM(5000)
                .alongWith(drive.alignToSpeaker())
                .andThen(intake.shoot(1).withTimeout(1))
                .andThen(shooter.runAtRPM(0));
    }

    public static Command stopShooting(Shooter shooter, Intake intake) {
        return shooter.runAtRPM(0).alongWith(intake.setShooter(0));
    }

}
