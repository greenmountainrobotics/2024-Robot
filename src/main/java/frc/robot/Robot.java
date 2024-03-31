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

import static frc.robot.constants.FieldConstants.SpeakerShootingDistance;
import static frc.robot.constants.IdConstants.PWMId.LedsId;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.Camera;
import frc.robot.subsystems.apriltagvision.AprilTagVision;
import frc.robot.subsystems.apriltagvision.photonvision.PhotonVision;
import frc.robot.subsystems.apriltagvision.photonvision.PhotonVisionIOReal;
import frc.robot.subsystems.apriltagvision.photonvision.PhotonVisionIOReplay;
import frc.robot.subsystems.apriltagvision.photonvision.PhotonVisionIOSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.imu.GyroIO;
import frc.robot.subsystems.drive.imu.GyroIOPigeon2;
import frc.robot.subsystems.drive.module.ModuleIO;
import frc.robot.subsystems.drive.module.ModuleIOReal;
import frc.robot.subsystems.drive.module.ModuleIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.leds.CustomLeds;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOReal;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.util.RunMode;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.urcl.URCL;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Auto auto;
  private DriverControl driverControl;

  public Drive drive;
  public AprilTagVision aprilTagVision;
  public Intake intake;
  public Shooter shooter;
  public Leds leds;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    initLogging();

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
            new AprilTagVision(new PhotonVision(new PhotonVisionIOReal(Camera.BackCamera)) /*,
                new PhotonVision(new PhotonVisionIOReal(Camera.FrontRightCamera)),
                new PhotonVision(new PhotonVisionIOReal(Camera.FrontLeftCamera))*/);
        intake = new Intake(new IntakeIOReal());
        shooter = new Shooter(new ShooterIOReal());
        leds = new Leds(new CustomLeds(LedsId));
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
                  new PhotonVision(new PhotonVisionIOSim(Camera.BackCamera, drive::getPose)),
                  new PhotonVision(new PhotonVisionIOSim(Camera.FrontRightCamera, drive::getPose)),
                  new PhotonVision(new PhotonVisionIOSim(Camera.FrontLeftCamera, drive::getPose)));
        } else {
          aprilTagVision =
              new AprilTagVision(
                  new PhotonVision(new PhotonVisionIOReal(Camera.BackCamera)),
                  new PhotonVision(new PhotonVisionIOReal(Camera.FrontRightCamera)),
                  new PhotonVision(new PhotonVisionIOReal(Camera.FrontLeftCamera)));
        }

        intake = new Intake(new IntakeIOSim());
        shooter = new Shooter(new ShooterIOSim());
        leds = new Leds(new AddressableLED(LedsId));
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
        aprilTagVision =
            new AprilTagVision(
                new PhotonVision(new PhotonVisionIOReplay(Camera.BackCamera)),
                new PhotonVision(new PhotonVisionIOReplay(Camera.FrontRightCamera)),
                new PhotonVision(new PhotonVisionIOReplay(Camera.FrontLeftCamera)));
        intake = new Intake(new IntakeIO() {});
        shooter = new Shooter(new ShooterIO() {});
        leds = new Leds(new AddressableLED(LedsId));
        break;
    }

    if (RunMode.getMode() == RunMode.SIM && Config.SIMULATE_CAMERAS) {
      aprilTagVision.setDataInterface((a, b) -> {}, drive::getPose);
    } else {
      aprilTagVision.setDataInterface(drive::addVisionMeasurement, drive::getPose);
    }

    leds.setDefaultCommand(
        new InstantCommand(
                () -> {
                  Leds.State.AutoEnabled = this.isAutonomousEnabled();
                  Leds.State.Enabled = this.isEnabled();
                },
                leds)
            .ignoringDisable(true));

    auto = new Auto(this);
    driverControl = new DriverControl(this);

    SmartDashboard.putNumber("amp speed", 130);
    SmartDashboard.putNumber("amp ratio", 18);
    SmartDashboard.putNumber("Shooting Distance M", SpeakerShootingDistance);
  }

  void initLogging() {
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);

    // Constants.Battery battery =
    // RobotBase.isReal() ? BatteryTracker.scanBatteryQR() : Constants.Battery.NONE;
    // Logger.recordMetadata("Battery", battery.year + "_" + battery);

    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers & replay source
    switch (RunMode.getMode()) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"), 0.02));
        break;
    }

    // Start AdvantageKit logger
    Logger.registerURCL(URCL.startExternal());
    Logger.start();
  }

  // DO NOT modify anything below this line.
  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    auto.schedule();
  }

  @Override
  public void teleopInit() {
    auto.cancel();
  }
}
