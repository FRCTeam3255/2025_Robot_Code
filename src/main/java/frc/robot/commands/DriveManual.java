// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;
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
import frc.robot.RobotContainer;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Drivetrain;

public class DriveManual extends Command {
  StateMachine subStateMachine;
  Drivetrain subDrivetrain;
  AlgaeIntake subAlgaeIntake;
  DoubleSupplier xAxis, yAxis, rotationAxis;
  BooleanSupplier slowMode, leftReef, rightReef, coralStationLeft, coralStationRight, processor, net, prepClimb;
  Elevator subElevator;
  boolean isOpenLoop;
  double redAllianceMultiplier = 1;
  double slowMultiplier = 0;
  Pose2d netPose, desiredNetPose;
  boolean netAlignStarted = false;
  Pose2d processorPose, desiredProcessorPose, desiredCage = new Pose2d();
  boolean processorAlignStarted = false;
  boolean prepClimbValid = false;

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
   * @param prepClimb
   */
  public DriveManual(StateMachine subStateMachine, Drivetrain subDrivetrain, Elevator subElevator,
      AlgaeIntake subAlgaeIntake, DoubleSupplier xAxis,
      DoubleSupplier yAxis,
      DoubleSupplier rotationAxis, BooleanSupplier slowMode, BooleanSupplier leftReef, BooleanSupplier rightReef,
      BooleanSupplier coralStationLeft, BooleanSupplier coralStationRight,
      BooleanSupplier processorBtn, BooleanSupplier net, BooleanSupplier prepClimb) {
    this.subStateMachine = subStateMachine;
    this.subDrivetrain = subDrivetrain;
    this.subAlgaeIntake = subAlgaeIntake;
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
    this.prepClimb = prepClimb;

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

    if (RobotContainer.isPracticeBot()) {
      AngularVelocity rVelocity = Units.RadiansPerSecond
          .of(rotationAxis.getAsDouble() * constDrivetrain.TURN_SPEED.in(Units.RadiansPerSecond)
              * elevatorHeightMultiplier);
    } else {
      AngularVelocity rVelocity = Units.RadiansPerSecond
        .of(-rotationAxis.getAsDouble() * constDrivetrain.TURN_SPEED.in(Units.RadiansPerSecond)
            * elevatorHeightMultiplier);
    }

    if (prepClimb.getAsBoolean()) {
      prepClimbValid = true;
      boolean onOpposingSide = subDrivetrain.getPose().getX() > 8.775;
      List<Pose2d> cagePoses = constField.getAllCagePositions(onOpposingSide).get();
      desiredCage = currentPose.nearest(cagePoses);
    } else if (subDrivetrain.driveBackwards
        && Math.abs(rotationAxis.getAsDouble()) < constControllers.DRIVER_LEFT_STICK_DEADBAND) {
      subDrivetrain.drive(
          new Translation2d(-0.4, 0),
          0.0,
          isOpenLoop, false);
    }
    // -- Controlling --
    else if (leftReef.getAsBoolean() || rightReef.getAsBoolean()) {
      netAlignStarted = false;
      processorAlignStarted = false;
      prepClimbValid = false;

      if (subStateMachine.inCleaningState()) {
        subDrivetrain.algaeAutoAlign(xVelocity, yVelocity, rVelocity, transMultiplier, isOpenLoop,
            Constants.constDrivetrain.TELEOP_AUTO_ALIGN.MAX_AUTO_DRIVE_ALGAE_DISTANCE, DriverState.ALGAE_AUTO_DRIVING,
            DriverState.ALGAE_ROTATION_SNAPPING, subStateMachine, false, false);
      } else {
        subDrivetrain.reefAutoAlign(leftReef.getAsBoolean(), xVelocity, yVelocity, rVelocity, transMultiplier,
            isOpenLoop,
            Constants.constDrivetrain.TELEOP_AUTO_ALIGN.MAX_AUTO_DRIVE_REEF_DISTANCE,
            DriverState.REEF_AUTO_DRIVING, DriverState.REEF_ROTATION_SNAPPING, subStateMachine, false, false);
      }
    }

    // -- Coral Station --
    else if (coralStationRight.getAsBoolean()) {
      netAlignStarted = false;
      processorAlignStarted = false;
      prepClimbValid = false;

      Pose2d desiredCoralStation = constField.getCoralStationPositions().get().get(0);

      subDrivetrain.rotationalAlign(desiredCoralStation, xVelocity, yVelocity, isOpenLoop,
          DriverState.CORAL_STATION_ROTATION_SNAPPING, subStateMachine);
    }

    else if (coralStationLeft.getAsBoolean()) {
      netAlignStarted = false;
      processorAlignStarted = false;
      prepClimbValid = false;

      Pose2d desiredCoralStation = constField.getCoralStationPositions().get().get(2);

      subDrivetrain.rotationalAlign(desiredCoralStation, xVelocity, yVelocity, isOpenLoop,
          DriverState.CORAL_STATION_ROTATION_SNAPPING, subStateMachine);
    }

    // -- Processors --
    else if (processor.getAsBoolean()) {
      netAlignStarted = false;
      prepClimbValid = false;
      boolean driverOverrideX = yVelocity.abs(Units.MetersPerSecond) > 0.1;

      if (!processorAlignStarted || driverOverrideX) {
        Pose2d processorPose = subDrivetrain.getDesiredProcessor();
        if (processorPose.equals(constField.getProcessorPositions().get().get(1))) {
          yVelocity = yVelocity.unaryMinus();
        }
        desiredProcessorPose = new Pose2d(processorPose.getX(), currentPose.getY(), processorPose.getRotation());
        processorAlignStarted = true;
      }

      Distance processorDistance = Units.Meters
          .of(currentPose.getTranslation().getDistance(desiredProcessorPose.getTranslation()));

      subDrivetrain.autoAlign(processorDistance, desiredProcessorPose, yVelocity.unaryMinus(), xVelocity.unaryMinus(),
          rVelocity,
          transMultiplier, isOpenLoop, Constants.constDrivetrain.TELEOP_AUTO_ALIGN.MAX_AUTO_DRIVE_PROCESSOR_DISTANCE,
          DriverState.PROCESSOR_AUTO_DRIVING, DriverState.PROCESSOR_ROTATION_SNAPPING, subStateMachine, driverOverrideX,
          false);
    }

    // -- Net --
    else if (net.getAsBoolean()) {
      processorAlignStarted = false;
      prepClimbValid = false;
      boolean driverOverrideY = yVelocity.abs(Units.MetersPerSecond) > 0.1;
      if (!netAlignStarted || driverOverrideY) {
        Pose2d netPose = currentPose.nearest(constField.POSES.NET_POSES);
        if (netPose.equals(constField.POSES.NET_POSES.get(1))) {
          yVelocity = yVelocity.unaryMinus();
        }
        desiredNetPose = new Pose2d(netPose.getX(), currentPose.getY(), netPose.getRotation());
        netAlignStarted = true;
      }

      Distance netDistance = Units.Meters
          .of(currentPose.getTranslation().getDistance(desiredNetPose.getTranslation()));

      subDrivetrain.autoAlign(netDistance, desiredNetPose, xVelocity, yVelocity, rVelocity,
          transMultiplier, isOpenLoop, Constants.constDrivetrain.TELEOP_AUTO_ALIGN.MAX_AUTO_DRIVE_NET_DISTANCE,
          DriverState.NET_AUTO_DRIVING, DriverState.NET_ROTATION_SNAPPING, subStateMachine, false, driverOverrideY);
    }

    // // -- Prep Climb --
    // else if (prepClimbValid) {
    // netAlignStarted = false;
    // processorAlignStarted = false;
    // if (Math.abs(rotationAxis.getAsDouble()) >
    // constControllers.DRIVER_LEFT_STICK_DEADBAND) {
    // prepClimbValid = false;
    // }
    // subDrivetrain.rotationalAlign(desiredCage, xVelocity, yVelocity, isOpenLoop,
    // DriverState.CAGE_ROTATION_SNAPPING, subStateMachine);
    // }

    else {
      netAlignStarted = false;
      processorAlignStarted = false;
      prepClimbValid = false;
      subDrivetrain.driveBackwards = false;

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
