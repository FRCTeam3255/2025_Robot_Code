// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.frcteam3255.utils.SN_Math;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.StateMachine.DriverState;
import frc.robot.Constants;
import frc.robot.Constants.*;
import frc.robot.subsystems.Drivetrain;

public class DriveManual extends Command {
  StateMachine subStateMachine;
  Drivetrain subDrivetrain;
  DoubleSupplier xAxis, yAxis, rotationAxis;
  BooleanSupplier slowMode, leftReef, rightReef, coralStationLeft, coralStationRight, processor, net;
  Elevator subElevator;
  boolean isOpenLoop;
  double redAllianceMultiplier = 1;
  double slowMultiplier = 0;
  Pose2d netPose, desiredNetPose;
  boolean netAlignStarted = false;

  /**
   * @param subStateMachine
   * @param subDrivetrain
   * @param subElevator
   * @param xAxis
   * @param yAxis
   * @param rotationAxis
   * @param slowMode
   * @param leftReef
   * @param rightReef
   * @param coralStationLeft
   * @param coralStationRight
   * @param processorBtn
   * @param net
   */
  public DriveManual(StateMachine subStateMachine, Drivetrain subDrivetrain, Elevator subElevator, DoubleSupplier xAxis,
      DoubleSupplier yAxis,
      DoubleSupplier rotationAxis, BooleanSupplier slowMode, BooleanSupplier leftReef, BooleanSupplier rightReef,
      BooleanSupplier coralStationLeft, BooleanSupplier coralStationRight,
      BooleanSupplier processorBtn, BooleanSupplier net) {
    this.subStateMachine = subStateMachine;
    this.subDrivetrain = subDrivetrain;
    this.xAxis = xAxis;
    this.yAxis = yAxis;
    this.rotationAxis = rotationAxis;
    this.slowMode = slowMode;
    this.leftReef = leftReef;
    this.rightReef = rightReef;
    this.coralStationLeft = coralStationLeft;
    this.coralStationRight = coralStationRight;
    this.subElevator = subElevator;
    this.processor = processorBtn;
    this.net = net;

    isOpenLoop = true;

    addRequirements(this.subDrivetrain);
  }

  @Override
  public void initialize() {
    redAllianceMultiplier = constField.isRedAlliance() ? -1 : 1;
  }

  @Override
  public void execute() {
    Pose2d currentPose = subDrivetrain.getPose();

    // -- Multipliers --
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

    double transMultiplier = slowMultiplier
        * constDrivetrain.OBSERVED_DRIVE_SPEED.in(Units.MetersPerSecond) * elevatorHeightMultiplier;

    // -- Velocities --
    LinearVelocity xVelocity = Units.MetersPerSecond.of(xAxis.getAsDouble() * transMultiplier);
    LinearVelocity yVelocity = Units.MetersPerSecond.of(-yAxis.getAsDouble() * transMultiplier);
    AngularVelocity rVelocity = Units.RadiansPerSecond
        .of(-rotationAxis.getAsDouble() * constDrivetrain.TURN_SPEED.in(Units.RadiansPerSecond)
            * elevatorHeightMultiplier);

    // -- Controlling --
    if (leftReef.getAsBoolean() || rightReef.getAsBoolean()) {
      netAlignStarted = false;
      // Reef auto-align is requested
      Pose2d desiredReef = subDrivetrain.getDesiredReef(leftReef.getAsBoolean());
      Distance reefDistance = Units.Meters
          .of(currentPose.getTranslation().getDistance(desiredReef.getTranslation()));

      // Begin reef auto align (rotationally, automatically driving, or w/ a driver
      // override)
      subDrivetrain.autoAlign(reefDistance, desiredReef, xVelocity, yVelocity, rVelocity, transMultiplier,
          isOpenLoop,
          Constants.constDrivetrain.TELEOP_AUTO_ALIGN.MAX_AUTO_DRIVE_REEF_DISTANCE,
          DriverState.REEF_AUTO_DRIVING, DriverState.REEF_ROTATION_SNAPPING, subStateMachine, false, false);

    }

    // -- Coral Station --
    else if (coralStationRight.getAsBoolean()) {
      netAlignStarted = false;
      Pose2d desiredCoralStation = constField.getCoralStationPositions().get().get(0);
      Distance coralStationDistance = Units.Meters
          .of(currentPose.getTranslation().getDistance(desiredCoralStation.getTranslation()));
      subDrivetrain.rotationalAutoAlign(coralStationDistance, desiredCoralStation, xVelocity, yVelocity, rVelocity,
          transMultiplier, isOpenLoop,
          Constants.constDrivetrain.TELEOP_AUTO_ALIGN.MAX_AUTO_DRIVE_CORAL_STATION_DISTANCE,
          DriverState.CORAL_STATION_AUTO_DRIVING, DriverState.CORAL_STATION_ROTATION_SNAPPING, subStateMachine);
    }

    else if (coralStationLeft.getAsBoolean()) {
      netAlignStarted = false;
      Pose2d desiredCoralStation = constField.getCoralStationPositions().get().get(2);
      Distance coralStationDistance = Units.Meters
          .of(currentPose.getTranslation().getDistance(desiredCoralStation.getTranslation()));
      subDrivetrain.rotationalAutoAlign(coralStationDistance, desiredCoralStation, xVelocity, yVelocity, rVelocity,
          transMultiplier, isOpenLoop,
          Constants.constDrivetrain.TELEOP_AUTO_ALIGN.MAX_AUTO_DRIVE_CORAL_STATION_DISTANCE,
          DriverState.CORAL_STATION_AUTO_DRIVING, DriverState.CORAL_STATION_ROTATION_SNAPPING, subStateMachine);
    }

    // -- Processors --
    else if (processor.getAsBoolean()) {
      netAlignStarted = false;
      Pose2d desiredProcessor = subDrivetrain.getDesiredProcessor();
      Distance processorDistance = Units.Meters
          .of(currentPose.getTranslation().getDistance(desiredProcessor.getTranslation()));

      subDrivetrain.rotationalAutoAlign(processorDistance, desiredProcessor, xVelocity, yVelocity, rVelocity,
          transMultiplier,
          isOpenLoop, Constants.constDrivetrain.TELEOP_AUTO_ALIGN.MAX_AUTO_DRIVE_PROCESSOR_DISTANCE,
          DriverState.PROCESSOR_AUTO_DRIVING, DriverState.PROCESSOR_ROTATION_SNAPPING,
          subStateMachine);
    }
    // -- Net --
    else if (net.getAsBoolean()) {

      boolean driverOverrideY = yVelocity.abs(Units.MetersPerSecond) > 0.1;
      if (!netAlignStarted || driverOverrideY) {
        Pose2d netPose = currentPose.nearest(constField.POSES.NET_POSES);
        desiredNetPose = new Pose2d(netPose.getX(), currentPose.getY(), netPose.getRotation());
        netAlignStarted = true;
      }

      Distance netDistance = Units.Meters
          .of(currentPose.getTranslation().getDistance(desiredNetPose.getTranslation()));

      subDrivetrain.autoAlign(netDistance, desiredNetPose, xVelocity, yVelocity.times(redAllianceMultiplier), rVelocity,
          transMultiplier, isOpenLoop, Constants.constDrivetrain.TELEOP_AUTO_ALIGN.MAX_AUTO_DRIVE_NET_DISTANCE,
          DriverState.NET_AUTO_DRIVING, DriverState.NET_ROTATION_SNAPPING, subStateMachine, false, driverOverrideY);
    } else {
      netAlignStarted = false;
      // Regular driving
      subDrivetrain.drive(
          new Translation2d(xVelocity.times(redAllianceMultiplier).in(Units.MetersPerSecond),
              yVelocity.times(redAllianceMultiplier).in(Units.MetersPerSecond)),
          rVelocity.in(Units.RadiansPerSecond), isOpenLoop);
      subStateMachine.setDriverState(DriverState.MANUAL);
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
