// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.frcteam3255.utils.SN_Math;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.Constants.*;
import frc.robot.subsystems.Drivetrain;

public class DriveManual extends Command {
  Drivetrain subDrivetrain;
  DoubleSupplier xAxis, yAxis, rotationAxis;
  BooleanSupplier slowMode, leftReef, rightReef, cageAlign;
  boolean isOpenLoop;
  double redAllianceMultiplier = 1;
  double slowMultiplier = 0;

  public DriveManual(Drivetrain subDrivetrain, DoubleSupplier xAxis, DoubleSupplier yAxis,
      DoubleSupplier rotationAxis, BooleanSupplier slowMode, BooleanSupplier leftReef, BooleanSupplier rightReef,
      BooleanSupplier cageAlign) {
    this.subDrivetrain = subDrivetrain;
    this.xAxis = xAxis;
    this.yAxis = yAxis;
    this.rotationAxis = rotationAxis;
    this.slowMode = slowMode;
    this.leftReef = leftReef;
    this.rightReef = rightReef;
    this.cageAlign = cageAlign;

    isOpenLoop = true;

    addRequirements(this.subDrivetrain);
  }

  @Override
  public void initialize() {
    redAllianceMultiplier = constField.isRedAlliance() ? -1 : 1;
  }

  @Override
  public void execute() {
    subDrivetrain.setFieldRelative();

    if (slowMode.getAsBoolean()) {
      slowMultiplier = constDrivetrain.SLOW_MODE_MULTIPLIER;
    } else {
      slowMultiplier = 1;
    }

    // Get Joystick inputs
    double elevatorHeightMultiplier = SN_Math.interpolate(
        subElevator.getElevatorPosition().in(Units.Meters),
        0.0, constElevator.MAX_HEIGHT.in(Units.Meters),
        1.0, constDrivetrain.MINIMUM_ELEVATOR_MULTIPLIER);

    double transMultiplier = slowMultiplier * redAllianceMultiplier
        * constDrivetrain.OBSERVED_DRIVE_SPEED.in(Units.MetersPerSecond) * elevatorHeightMultiplier;

    LinearVelocity xVelocity = Units.MetersPerSecond.of(xAxis.getAsDouble() * transMultiplier);
    LinearVelocity yVelocity = Units.MetersPerSecond.of(-yAxis.getAsDouble() * transMultiplier);
    AngularVelocity rVelocity = Units.RadiansPerSecond
        .of(-rotationAxis.getAsDouble() * constDrivetrain.TURN_SPEED.in(Units.RadiansPerSecond)
            * elevatorHeightMultiplier);

    // Cage auto-align
    if (cageAlign.getAsBoolean()) {
      Pose2d desiredCage = subDrivetrain.getDesiredCage();
      Distance cageDistance = Units.Meters
          .of(subDrivetrain.getPose().getTranslation().getDistance(desiredCage.getTranslation()));

      ChassisSpeeds desiredChassisSpeeds = subDrivetrain.getAlignmentSpeeds(desiredCage);
      subDrivetrain.drive(desiredChassisSpeeds, isOpenLoop);
    }

    // Reef auto-align
    if (leftReef.getAsBoolean() || rightReef.getAsBoolean()) {
      Pose2d desiredReef = subDrivetrain.getDesiredReef(leftReef.getAsBoolean());
      Distance reefDistance = Units.Meters
          .of(subDrivetrain.getPose().getTranslation().getDistance(desiredReef.getTranslation()));

      if (reefDistance.gte(constDrivetrain.TELEOP_AUTO_ALIGN.MAX_AUTO_DRIVE_DISTANCE)) {
        // Rotational-only auto-align
        subDrivetrain.drive(new Translation2d(xVelocity.in(Units.MetersPerSecond), yVelocity.in(Units.MetersPerSecond)),
            subDrivetrain.getVelocityToRotate(desiredReef.getRotation()).in(Units.RadiansPerSecond), isOpenLoop);

        // Auto-align Driver Override
      } else if (xVelocity.gte(constDrivetrain.TELEOP_AUTO_ALIGN.MIN_DRIVER_OVERRIDE)) {
        // Assumes that the driver only overrides if we're already facing the reef,
        // so it can be robot relative to make things ezpz :>
        subDrivetrain.setRobotRelative();
        subDrivetrain.drive(new Translation2d(xVelocity.in(Units.MetersPerSecond), yVelocity.in(Units.MetersPerSecond)),
            rVelocity.in(RadiansPerSecond), isOpenLoop);
      } else {
        ChassisSpeeds desiredChassisSpeeds = subDrivetrain.getAlignmentSpeeds(desiredReef);
        subDrivetrain.drive(desiredChassisSpeeds, isOpenLoop);
      }
    } else {
      // Regular driving
      subDrivetrain.drive(new Translation2d(xVelocity.in(Units.MetersPerSecond), yVelocity.in(Units.MetersPerSecond)),
          rVelocity.in(RadiansPerSecond), isOpenLoop);
    }

  }

  @Override
  public void end(boolean interrupted) {
    subDrivetrain.neutralDriveOutputs();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
