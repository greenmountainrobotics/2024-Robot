// Copyright 2021-2023 FRC 6328
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

package frc.robot.subsystems.drive.module;

import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;
import static frc.robot.Constants.CANIdConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drive.SparkFlexOdometryThread;
import java.util.Queue;

/**
 * Module IO implementation for SparkFlex drive motor controller, SparkFlex turn motor controller
 * (NEO or NEO 550), and analog absolute encoder connected to the RIO
 *
 * <p>NOTE: This implementation should be used as a starting point and adapted to different hardware
 * configurations (e.g. If using a CANcoder, copy from "ModuleIOTalonFX")
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIOSparkFlex implements ModuleIO {
  // Gear ratios for SDS MK4i L3
  private static final double DRIVE_GEAR_RATIO = (50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0);
  private static final double TURN_GEAR_RATIO = 150.0 / 7.0;

  private final CANSparkFlex driveSparkFlex;
  private final CANSparkFlex turnSparkFlex;

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turnRelativeEncoder;
  private final CANcoder cancoder;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;
  private final Queue<Double> timestampQueue;

  private final StatusSignal<Double> turnAbsolutePosition;

  private final boolean isTurnMotorInverted = true;
  private final Rotation2d absoluteEncoderOffset;

  private final String positionName;

  public ModuleIOSparkFlex(int index) {
    switch (index) {
      case 0:
        // front left
        driveSparkFlex = new CANSparkFlex(FrontLeftDriveId, kBrushless);
        turnSparkFlex = new CANSparkFlex(FrontLeftTurnId, kBrushless);
        cancoder = new CANcoder(FrontLeftEncoderId);
        absoluteEncoderOffset = new Rotation2d(-4.5345926536); // CALIBRATED
        positionName = "FrontLeft";
        break;
      case 1:
        // front right
        driveSparkFlex = new CANSparkFlex(FrontRightDriveId, kBrushless);
        turnSparkFlex = new CANSparkFlex(FrontRightTurnId, kBrushless);
        cancoder = new CANcoder(FrontRightEncoderId);
        absoluteEncoderOffset = new Rotation2d(-2.326); // CALIBRATED
        positionName = "FrontRight";
        break;
      case 2:
        // back left
        driveSparkFlex = new CANSparkFlex(BackLeftDriveId, kBrushless);
        turnSparkFlex = new CANSparkFlex(BackLeftTurnId, kBrushless);
        cancoder = new CANcoder(BackLeftEncoderId);
        absoluteEncoderOffset = new Rotation2d(-3.8085926536); // CALIBRATED
        positionName = "BackLeft";
        break;
      case 3:
        // back right
        driveSparkFlex = new CANSparkFlex(BackRightDriveId, kBrushless);
        turnSparkFlex = new CANSparkFlex(BackRightTurnId, kBrushless);
        cancoder = new CANcoder(BackRightEncoderId);
        absoluteEncoderOffset = new Rotation2d(-0.907); // CALIBRATED
        positionName = "BackRight";
        break;
      default:
        throw new RuntimeException("Invalid module index");
    }

    turnAbsolutePosition = cancoder.getAbsolutePosition();
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, turnAbsolutePosition);

    driveSparkFlex.restoreFactoryDefaults();
    turnSparkFlex.restoreFactoryDefaults();

    driveSparkFlex.setCANTimeout(250);
    turnSparkFlex.setCANTimeout(250);

    driveEncoder = driveSparkFlex.getEncoder();
    turnRelativeEncoder = turnSparkFlex.getEncoder();

    turnSparkFlex.setInverted(isTurnMotorInverted);
    driveSparkFlex.setSmartCurrentLimit(40);
    turnSparkFlex.setSmartCurrentLimit(30);
    driveSparkFlex.enableVoltageCompensation(12.0);
    turnSparkFlex.enableVoltageCompensation(12.0);

    driveEncoder.setPosition(0.0);
    driveEncoder.setMeasurementPeriod(10);
    driveEncoder.setAverageDepth(2);

    turnRelativeEncoder.setPosition(0.0);
    turnRelativeEncoder.setMeasurementPeriod(10);
    turnRelativeEncoder.setAverageDepth(2);

    driveSparkFlex.setCANTimeout(0);
    turnSparkFlex.setCANTimeout(0);

    driveSparkFlex.setPeriodicFramePeriod(
        PeriodicFrame.kStatus2, (int) (1000.0 / Module.ODOMETRY_FREQUENCY));
    turnSparkFlex.setPeriodicFramePeriod(
        PeriodicFrame.kStatus2, (int) (1000.0 / Module.ODOMETRY_FREQUENCY));
    drivePositionQueue =
        SparkFlexOdometryThread.getInstance().registerSignal(driveEncoder::getPosition);
    turnPositionQueue =
        SparkFlexOdometryThread.getInstance().registerSignal(turnRelativeEncoder::getPosition);

    driveSparkFlex.burnFlash();
    turnSparkFlex.burnFlash();

    timestampQueue = SparkFlexOdometryThread.getInstance().makeTimestampQueue();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    BaseStatusSignal.refreshAll(turnAbsolutePosition);

    inputs.drivePositionRad =
        Units.rotationsToRadians(driveEncoder.getPosition()) / DRIVE_GEAR_RATIO;
    inputs.driveVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(driveEncoder.getVelocity()) / DRIVE_GEAR_RATIO;
    inputs.driveAppliedVolts = driveSparkFlex.getAppliedOutput() * driveSparkFlex.getBusVoltage();
    inputs.driveCurrentAmps = new double[] {driveSparkFlex.getOutputCurrent()};

    inputs.turnAbsolutePosition =
        Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble())
            .minus(absoluteEncoderOffset);
    inputs.turnPosition =
        Rotation2d.fromRotations(turnRelativeEncoder.getPosition() / TURN_GEAR_RATIO);
    inputs.turnVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(turnRelativeEncoder.getVelocity())
            / TURN_GEAR_RATIO;
    inputs.turnAppliedVolts = turnSparkFlex.getAppliedOutput() * turnSparkFlex.getBusVoltage();
    inputs.turnCurrentAmps = new double[] {turnSparkFlex.getOutputCurrent()};

    inputs.odometryDrivePositionsRad =
        drivePositionQueue.stream()
            .mapToDouble((Double value) -> Units.rotationsToRadians(value) / DRIVE_GEAR_RATIO)
            .toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromRotations(value / TURN_GEAR_RATIO))
            .toArray(Rotation2d[]::new);
    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();

    inputs.driveMotorTemperature = driveSparkFlex.getMotorTemperature();
    inputs.turnMotorTemperature = turnSparkFlex.getMotorTemperature();

    drivePositionQueue.clear();
    turnPositionQueue.clear();
    timestampQueue.clear();
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveSparkFlex.setVoltage(volts);
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnSparkFlex.setVoltage(volts);
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    driveSparkFlex.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    turnSparkFlex.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }
}
