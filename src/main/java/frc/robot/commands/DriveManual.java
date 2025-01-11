// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.constDrivetrain;
import frc.robot.Constants.constField;
import frc.robot.subsystems.Drivetrain;

public class DriveManual extends Command {
  Drivetrain subDrivetrain;
  DoubleSupplier xAxis, yAxis, rotationAxis;
  BooleanSupplier slowMode;
  boolean isOpenLoop;
  double redAllianceMultiplier = 1;
  double slowMultiplier = 0;

  public DriveManual(Drivetrain subDrivetrain, DoubleSupplier xAxis, DoubleSupplier yAxis,
      DoubleSupplier rotationAxis, BooleanSupplier slowMode) {
    this.subDrivetrain = subDrivetrain;
    this.xAxis = xAxis;
    this.yAxis = yAxis;
    this.rotationAxis = rotationAxis;
    this.slowMode = slowMode;

    isOpenLoop = true;

    addRequirements(this.subDrivetrain);
  }

  @Override
  public void initialize() {
    redAllianceMultiplier = constField.isRedAlliance() ? -1 : 1;
  }

  @Override
  public void execute() {
    if (slowMode.getAsBoolean()) {
      slowMultiplier = constDrivetrain.SLOW_MODE_MULTIPLIER;
    } else {
      slowMultiplier = 1;
    }

    // Get Joystick inputs
    double transMultiplier = slowMultiplier * redAllianceMultiplier
        * constDrivetrain.OBSERVED_DRIVE_SPEED.in(Units.MetersPerSecond);

    double xVelocity = xAxis.getAsDouble() * transMultiplier;
    double yVelocity = -yAxis.getAsDouble() * transMultiplier;
    double rVelocity = -rotationAxis.getAsDouble() * constDrivetrain.TURN_SPEED.in(Units.RadiansPerSecond);

    subDrivetrain.drive(new Translation2d(xVelocity, yVelocity), rVelocity, isOpenLoop);
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
