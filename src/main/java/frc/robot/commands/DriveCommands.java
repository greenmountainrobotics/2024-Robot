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

package frc.robot.commands;

import static frc.robot.Constants.FieldConstants.*;
import static frc.robot.Constants.RobotConstants.WidthWithBumpersX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.subsystems.drive.Drive;
import java.util.Set;
import java.util.function.DoubleSupplier;

public class DriveCommands {
  private static final double DEADBAND = 0.25;

  private DriveCommands() {}

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          // Apply deadband
          double linearMagnitude =
              MathUtil.applyDeadband(
                  Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), DEADBAND);
          Rotation2d linearDirection =
              new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Square values
          linearMagnitude = linearMagnitude * linearMagnitude;
          omega = Math.copySign(omega * omega, omega);

          // Calcaulate new linear velocity
          Translation2d linearVelocity =
              new Pose2d(new Translation2d(), linearDirection)
                  .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                  .getTranslation();

          // Convert to field relative speeds & send command
          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec(),
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
        },
        drive);
  }

  public static Command alignToAmp() {
    return new DeferredCommand(
        () -> {
          Pose2d pose =
              new Pose2d(
                  AmpCenter.minus(
                      new Translation2d(WidthWithBumpersX, 0)
                          .times(0.5)
                          .rotateBy(Rotation2d.fromDegrees(90))),
                  AmpRotation.plus(Rotation2d.fromDegrees(180)));
          if (DriverStation.getAlliance().isPresent()
              && DriverStation.getAlliance().get() == Alliance.Red) pose = Drive.flipPose(pose);
          return Drive.runToPose(pose);
        },
        Set.of());
  }

  public static Command alignToSource() {
    return new DeferredCommand(
        () -> {
          Pose2d pose =
              new Pose2d(
                  SourceCloseSideCorner.plus(SourceFarSideCorner)
                      .div(2)
                      .plus(
                          new Translation2d(WidthWithBumpersX, 0)
                              .times(0.5)
                              .rotateBy(SourceRotation)),
                  SourceRotation.plus(Rotation2d.fromDegrees(180)));
          if (DriverStation.getAlliance().isPresent()
              && DriverStation.getAlliance().get() == Alliance.Red) pose = Drive.flipPose(pose);
          return Drive.runToPose(pose);
        },
        Set.of());
  }
}
