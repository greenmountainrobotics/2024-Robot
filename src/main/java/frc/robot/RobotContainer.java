// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveCommands;
import frc.robot.constants.Camera;
import frc.robot.subsystems.apriltagvision.*;
import frc.robot.subsystems.apriltagvision.photonvision.PhotonVision;
import frc.robot.subsystems.apriltagvision.photonvision.PhotonVisionIO;
import frc.robot.subsystems.apriltagvision.photonvision.PhotonVisionIOReal;
import frc.robot.subsystems.apriltagvision.photonvision.PhotonVisionIOSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.imu.GyroIO;
import frc.robot.subsystems.drive.imu.GyroIOPigeon2;
import frc.robot.subsystems.drive.module.ModuleIO;
import frc.robot.subsystems.drive.module.ModuleIOSim;
import frc.robot.subsystems.drive.module.ModuleIOReal;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOReal;
import frc.robot.util.RunMode;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final AprilTagVision aprilTagVision;
  private final Intake intake;
  private final Shooter shooter;

  // Controller
  private final CommandXboxController controller1 = new CommandXboxController(0);
  private final CommandXboxController controller2 = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final LoggedDashboardNumber autoDelay;

  private final LoggedDashboardNumber flywheelSpeedInput =
      new LoggedDashboardNumber("Flywheel Speed", 1500.0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (RunMode.getMode()) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOReal(0),
                new ModuleIOReal(1),
                new ModuleIOReal(2),
                new ModuleIOReal(3));
        aprilTagVision =
            new AprilTagVision(new PhotonVision(new PhotonVisionIOReal(Camera.BackCamera)));
        intake = new Intake(new IntakeIOReal());
        shooter = new Shooter(new ShooterIOReal());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());

        if (Config.SIMULATE_CAMERAS) {
          aprilTagVision =
              new AprilTagVision(
                  new PhotonVision(new PhotonVisionIOSim(Camera.BackCamera, drive::getPose)));
        } else {
          aprilTagVision =
              new AprilTagVision(new PhotonVision(new PhotonVisionIOReal(Camera.BackCamera)));
        }

        intake = new Intake(new IntakeIOSim());
        shooter = new Shooter(new ShooterIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        aprilTagVision = new AprilTagVision(new PhotonVision(new PhotonVisionIO() {}));
        intake = new Intake(new IntakeIO() {});
        shooter = new Shooter(new ShooterIO() {});
        break;
    }

    if (RunMode.getMode() == RunMode.SIM && Config.SIMULATE_CAMERAS) {
      aprilTagVision.setDataInterface((a, b) -> {}, drive::getPose);
    } else {
      aprilTagVision.setDataInterface(drive::addVisionMeasurement, drive::getPose);
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices");
    autoDelay = new LoggedDashboardNumber("Auto Delay", 0);
    configureAutos();

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller1.getLeftY(),
            () -> -controller1.getLeftX(),
            () -> -controller1.getRightX()));

    controller1
        .y()
        .onTrue(
            new SequentialCommandGroup(
                Commands.runOnce(() -> intake.setArticulation(Rotation2d.fromDegrees(-90)), intake),
                Commands.runOnce(() -> intake.setExtension(1), intake)));
    controller1
        .x()
        .onTrue(
            new SequentialCommandGroup(
                Commands.runOnce(() -> intake.setArticulation(new Rotation2d()), intake),
                Commands.runOnce(() -> intake.setExtension(0), intake)));
    controller1.a().whileTrue(DriveCommands.alignToAmp(drive));
    controller1.b().whileTrue(DriveCommands.alignToSource(drive));
  }

  private void configureAutos() {
    // No-op
    autoChooser.addDefaultOption("None", new InstantCommand(() -> {}));

    // Autos
    autoChooser.addOption("Close side to Amp", Autos.CloseSideToAmp(drive, shooter));
    autoChooser.addOption("Far side to Amp", Autos.FarSideToAmp(drive, shooter));
    autoChooser.addOption(
        "Close side to Amp to Source", Autos.CloseSideToAmpToSource(drive, shooter));
    autoChooser.addOption("Far side to Amp to Source", Autos.FarSideToAmpToSource(drive, shooter));
    autoChooser.addOption(
        "Close side to Amp to Middle", Autos.CloseSideToAmpToMiddle(drive, shooter));
    autoChooser.addOption("Far side to Amp to Middle", Autos.FarSideToAmpToMiddle(drive, shooter));
    autoChooser.addOption("Far side to Source", Autos.FarSideToSource(drive));
    autoChooser.addOption("Shoot in Amp", Autos.ShootInAmp(shooter));

    // SysId Routines
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    if (autoDelay.get() != 0) {
      return new SequentialCommandGroup(new WaitCommand(autoDelay.get()), autoChooser.get());
    } else {
      return autoChooser.get();
    }
  }
}
