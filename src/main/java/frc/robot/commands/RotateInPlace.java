// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.constDrivetrain;
import frc.robot.subsystems.Drivetrain;

public class RotateInPlace extends Command {
  Drivetrain subDrivetrain;
  Angle desiredRotation;
  AngularVelocity rVelocity;
  final boolean isOpenLoop = false;

  /**
   * Rotates the robot in place to face a given Field-Relative rotation.
   * 
   * @param subDrivetrain   The instance of the subsystem we are making run this
   *                        command
   * @param desiredRotation The desired angle you would like to rotate towards
   *                        (Field-Relative Angle)
   * 
   * @see <a href=
   *      "https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html">Field
   *      Coordinate System</a>
   * 
   */
  public RotateInPlace(Drivetrain subDrivetrain, Angle desiredRotation) {
    this.subDrivetrain = subDrivetrain;
    this.desiredRotation = desiredRotation;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subDrivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    rVelocity = subDrivetrain.getVelocityToRotate(desiredRotation);

    subDrivetrain.drive(new Translation2d(0, 0), rVelocity.in(Units.RadiansPerSecond), isOpenLoop);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subDrivetrain.drive(new Translation2d(0, 0), 0, isOpenLoop);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return subDrivetrain.getRotationMeasure().isNear(desiredRotation,
        constDrivetrain.AT_ROTATION_TOLERANCE);
  }
}
