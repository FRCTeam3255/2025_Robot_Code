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
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
  BooleanSupplier slowMode, leftReef, rightReef, leftCoralStationNear, rightCoralStationNear, leftCoralStationFar,
      rightCoralStationFar, processor, algae;
  Elevator subElevator;
  boolean isOpenLoop;
  double redAllianceMultiplier = 1;
  double slowMultiplier = 0;

  public DriveManual(StateMachine subStateMachine, Drivetrain subDrivetrain, Elevator subElevator, DoubleSupplier xAxis,
      DoubleSupplier yAxis,
      DoubleSupplier rotationAxis, BooleanSupplier slowMode, BooleanSupplier leftReef, BooleanSupplier rightReef,
      BooleanSupplier leftCoralStationNear, BooleanSupplier rightCoralStationNear, BooleanSupplier leftCoralStationFar,
      BooleanSupplier rightCoralStationFar, BooleanSupplier processorBtn, BooleanSupplier algae) {
    this.subStateMachine = subStateMachine;
    this.subDrivetrain = subDrivetrain;
    this.xAxis = xAxis;
    this.yAxis = yAxis;
    this.rotationAxis = rotationAxis;
    this.slowMode = slowMode;
    this.leftReef = leftReef;
    this.rightReef = rightReef;
    this.leftCoralStationNear = leftCoralStationNear;
    this.rightCoralStationNear = rightCoralStationNear;
    this.leftCoralStationFar = leftCoralStationFar;
    this.rightCoralStationFar = rightCoralStationFar;
    this.subElevator = subElevator;
    this.processor = processorBtn;
    this.algae = algae;

    isOpenLoop = true;

    addRequirements(this.subDrivetrain);
  }

  @Override
  public void initialize() {
    redAllianceMultiplier = constField.isRedAlliance() ? -1 : 1;
  }

  @Override
  public void execute() {
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

    double transMultiplier = slowMultiplier * redAllianceMultiplier
        * constDrivetrain.OBSERVED_DRIVE_SPEED.in(Units.MetersPerSecond) * elevatorHeightMultiplier;

    // -- Velocities --
    LinearVelocity xVelocity = Units.MetersPerSecond.of(xAxis.getAsDouble() * transMultiplier);
    LinearVelocity yVelocity = Units.MetersPerSecond.of(-yAxis.getAsDouble() * transMultiplier);
    AngularVelocity rVelocity = Units.RadiansPerSecond
        .of(-rotationAxis.getAsDouble() * constDrivetrain.TURN_SPEED.in(Units.RadiansPerSecond)
            * elevatorHeightMultiplier);

    // -- Coral Station --
    if (leftCoralStationFar.getAsBoolean()) {
      Pose2d desiredCoralStation = Constants.constField.POSES.LEFT_CORAL_STATION_FAR;
      Distance coralStationDistance = Units.Meters
          .of(subDrivetrain.getPose().getTranslation().getDistance(desiredCoralStation.getTranslation()));

      subDrivetrain.autoAlign(coralStationDistance, desiredCoralStation, xVelocity, yVelocity, rVelocity,
          transMultiplier, isOpenLoop,
          Constants.constDrivetrain.TELEOP_AUTO_ALIGN.MAX_AUTO_DRIVE_CORAL_STATION_DISTANCE,
          DriverState.CORAL_STATION_AUTO_DRIVING, DriverState.CORAL_STATION_ROTATION_SNAPPING, subStateMachine);
    }

    else if (leftCoralStationNear.getAsBoolean()) {
      Pose2d desiredCoralStation = Constants.constField.POSES.LEFT_CORAL_STATION_NEAR;
      Distance coralStationDistance = Units.Meters
          .of(subDrivetrain.getPose().getTranslation().getDistance(desiredCoralStation.getTranslation()));

      subDrivetrain.autoAlign(coralStationDistance, desiredCoralStation, xVelocity, yVelocity, rVelocity,
          transMultiplier, isOpenLoop,
          Constants.constDrivetrain.TELEOP_AUTO_ALIGN.MAX_AUTO_DRIVE_CORAL_STATION_DISTANCE,
          DriverState.CORAL_STATION_AUTO_DRIVING, DriverState.CORAL_STATION_ROTATION_SNAPPING, subStateMachine);
    }

    else if (rightCoralStationFar.getAsBoolean()) {
      Pose2d desiredCoralStation = Constants.constField.POSES.RIGHT_CORAL_STATION_FAR;
      Distance coralStationDistance = Units.Meters
          .of(subDrivetrain.getPose().getTranslation().getDistance(desiredCoralStation.getTranslation()));

      subDrivetrain.autoAlign(coralStationDistance, desiredCoralStation, xVelocity, yVelocity, rVelocity,
          transMultiplier, isOpenLoop,
          Constants.constDrivetrain.TELEOP_AUTO_ALIGN.MAX_AUTO_DRIVE_CORAL_STATION_DISTANCE,
          DriverState.CORAL_STATION_AUTO_DRIVING, DriverState.CORAL_STATION_ROTATION_SNAPPING, subStateMachine);
    }

    else if (rightCoralStationNear.getAsBoolean()) {
      Pose2d desiredCoralStation = Constants.constField.POSES.RIGHT_CORAL_STATION_NEAR;
      Distance coralStationDistance = Units.Meters
          .of(subDrivetrain.getPose().getTranslation().getDistance(desiredCoralStation.getTranslation()));

      subDrivetrain.autoAlign(coralStationDistance, desiredCoralStation, xVelocity, yVelocity, rVelocity,
          transMultiplier, isOpenLoop,
          Constants.constDrivetrain.TELEOP_AUTO_ALIGN.MAX_AUTO_DRIVE_CORAL_STATION_DISTANCE,
          DriverState.CORAL_STATION_AUTO_DRIVING, DriverState.CORAL_STATION_ROTATION_SNAPPING, subStateMachine);
    }

    // -- Controlling --
    else if (leftReef.getAsBoolean() || rightReef.getAsBoolean()) {
      // Reef auto-align is requested
      Pose2d desiredReef = subDrivetrain.getDesiredReef(leftReef.getAsBoolean());
      Distance reefDistance = Units.Meters
          .of(subDrivetrain.getPose().getTranslation().getDistance(desiredReef.getTranslation()));

      // Begin reef auto align (rotationally, automatically driving, or w/ a driver
      // override)
      subDrivetrain.autoAlign(reefDistance, desiredReef, xVelocity, yVelocity, rVelocity, transMultiplier,
          isOpenLoop,
          Constants.constDrivetrain.TELEOP_AUTO_ALIGN.MAX_AUTO_DRIVE_REEF_DISTANCE,
          DriverState.REEF_AUTO_DRIVING, DriverState.REEF_ROTATION_SNAPPING, subStateMachine);
      ;
    } else if (processor.getAsBoolean()) {
      Pose2d desiredProcessor = Constants.constField.POSES.PROCESSOR;
      Distance processorDistance = Units.Meters
          .of(subDrivetrain.getPose().getTranslation().getDistance(desiredProcessor.getTranslation()));
      subDrivetrain.autoAlign(processorDistance, desiredProcessor, xVelocity, yVelocity, rVelocity, transMultiplier,
          isOpenLoop, Constants.constDrivetrain.TELEOP_AUTO_ALIGN.MAX_AUTO_DRIVE_PROCESSOR_DISTANCE,
          DriverState.PROCESSOR_AUTO_DRIVING, DriverState.PROCESSOR_ROTATION_SNAPPING, subStateMachine

      );
    }

    else if (algae.getAsBoolean()) {
      Pose2d desiredAlgae = subDrivetrain.getDesiredAlgae();
      Distance algaeDistance = Units.Meters
          .of(subDrivetrain.getPose().getTranslation().getDistance(desiredAlgae.getTranslation()));
      subDrivetrain.autoAlign(algaeDistance, desiredAlgae, xVelocity, yVelocity, rVelocity, transMultiplier,
          isOpenLoop, Constants.constDrivetrain.TELEOP_AUTO_ALIGN.MAX_AUTO_DRIVE_ALGAE_DISTANCE,
          DriverState.ALGAE_AUTO_DRIVING, DriverState.ALGAE_ROTATION_SNAPPING, subStateMachine);

    }

    else {
      // Regular driving
      subDrivetrain.drive(new Translation2d(xVelocity.in(Units.MetersPerSecond), yVelocity.in(Units.MetersPerSecond)),
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
