// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import frc.robot.Robot;

public class RobotPoses extends SubsystemBase {

  Pose3d elevatorStageOne = Pose3d.kZero;
  Pose3d elevatorCarriage = Pose3d.kZero;
  Pose3d algaeIntake = Pose3d.kZero;
  private final Elevator subElevator = new Elevator();
  private final AlgaeIntake subAlgaeIntake = new AlgaeIntake();

  /** Creates a new RobotPoses. */
  public RobotPoses() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    double elevatorPos, algaeAngle;

    // If we're in simulation, we can't log real mechanism data because they don't
    // exist. Instead, we'll log where we *want* the mechanisms to be and assume
    // they get there instantly.
    if (Robot.isSimulation()) {
      elevatorPos = subElevator.getLastDesiredPosition().in(Units.Meters) / 2;
      algaeAngle = subAlgaeIntake.getLastDesiredPivotAngle().in(Units.Degrees);
    } else {
      // Use real positions
      elevatorPos = (subElevator.getElevatorPosition().in(Units.Meters) / 2);
      algaeAngle = subAlgaeIntake.getPivotAngle().in(Units.Degrees);
    }

    elevatorStageOne = new Pose3d(new Translation3d(0.0889,
        0,
        0.109474 + elevatorPos), Rotation3d.kZero);

    elevatorCarriage = elevatorStageOne
        .transformBy(new Transform3d(new Translation3d(0, 0, Units.Inches.of(1).in(Units.Meters) + elevatorPos),
            Rotation3d.kZero));

    algaeIntake = elevatorCarriage
        .transformBy(new Transform3d(new Translation3d(0.075438, 0, 0.292354), new Rotation3d(0, algaeAngle, 0)));

  }
}
