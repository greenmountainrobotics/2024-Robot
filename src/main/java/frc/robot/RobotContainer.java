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

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.apriltagvision.AprilTagVision;
import frc.robot.subsystems.apriltagvision.PhotonVision;
import frc.robot.subsystems.apriltagvision.PhotonVisionIO;
import frc.robot.subsystems.apriltagvision.PhotonVisionIOPhotonVision;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.imu.GyroIO;
import frc.robot.subsystems.drive.imu.GyroIOPigeon2;
import frc.robot.subsystems.drive.module.ModuleIO;
import frc.robot.subsystems.drive.module.ModuleIOSim;
import frc.robot.subsystems.drive.module.ModuleIOSparkFlex;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOSparkMax;
import frc.robot.subsystems.shooter.ShooterSimple;
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
  private final ShooterSimple shooter;

  // Controller
  private final CommandXboxController controller1 = new CommandXboxController(0);
  private final CommandXboxController controller2 = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final LoggedDashboardNumber flywheelSpeedInput =
      new LoggedDashboardNumber("Flywheel Speed", 1500.0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.getMode()) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOSparkFlex(0),
                new ModuleIOSparkFlex(1),
                new ModuleIOSparkFlex(2),
                new ModuleIOSparkFlex(3));
        aprilTagVision =
            new PhotonVision(
                new PhotonVisionIOPhotonVision("Arducam_OV2311_USB_Camera", drive::getPose));
        shooter = new ShooterSimple(new ShooterIOSparkMax());
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
        aprilTagVision =
            new PhotonVision(
                new PhotonVisionIOPhotonVision("Arducam_OV2311_USB_Camera", drive::getPose));
        shooter = new ShooterSimple(new ShooterIOSim());
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
        aprilTagVision = new PhotonVision(new PhotonVisionIO() {});
        shooter = new ShooterSimple(new ShooterIO() {});
        break;
    }

    aprilTagVision.setDataInterface(drive::addVisionMeasurement);

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices");
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

    controller1.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
    controller1.a().whileTrue(DriveCommands.alignToAmp(drive));
    controller1.b().whileTrue(DriveCommands.alignToSource(drive));

    shooter.setDefaultCommand(
        new RunCommand(
            () -> {
              double amt = Math.abs(controller2.getLeftY()) > 0.2 ? controller2.getLeftY() : 0;
              amt = amt > 0 ? amt * 0.1 : amt;
              shooter.setFlywheels(amt, -amt);
            },
            shooter));
  }

  private void configureAutos() {
    // No-op
    autoChooser.addOption("None", new InstantCommand(() -> {}));

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
    return autoChooser.get();
  }
}
