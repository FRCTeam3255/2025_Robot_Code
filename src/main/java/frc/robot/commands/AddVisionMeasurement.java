// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.frcteam3255.utils.LimelightHelpers;
import com.frcteam3255.utils.LimelightHelpers.PoseEstimate;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class AddVisionMeasurement extends Command {
  Drivetrain subDrivetrain;
  Vision subVision;

  PoseEstimate estimatedPose;
  double drivetrainRotation = 0;

  public AddVisionMeasurement(Drivetrain subDrivetrain, Vision subVision) {
    this.subDrivetrain = subDrivetrain;
    this.subVision = subVision;

    addRequirements(subVision);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    // Tells the limelight where we are on the fielSd
    LimelightHelpers.SetRobotOrientation("limelight",
        subDrivetrain.getPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);

    estimatedPose = subVision.getPoseEstimate();
    AngularVelocity gyroRate = Units.DegreesPerSecond.of(subDrivetrain.getGyroRate());

    if (!subVision.rejectUpdate(estimatedPose, gyroRate)) {
      subDrivetrain.addVisionMeasurement(estimatedPose.pose, estimatedPose.timestampSeconds);
    }
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
