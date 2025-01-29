// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import frc.robot.Robot;

@Logged
public class RobotPoses extends SubsystemBase {

  Pose3d comp0Drivetrain = Pose3d.kZero;
  Pose3d comp1ElevatorStageOne = Pose3d.kZero;
  Pose3d comp2ElevatorCarriage = Pose3d.kZero;
  Pose3d comp3AlgaeIntake = Pose3d.kZero;
  @NotLogged
  private Elevator subElevator;
  @NotLogged
  private AlgaeIntake subAlgaeIntake;
  @NotLogged
  private Drivetrain subDrivetrain;

  /** Creates a new RobotPoses. */

  public RobotPoses(Drivetrain subDrivetrain, Elevator subElevator, AlgaeIntake subAlgaeIntake) {
    this.subDrivetrain = subDrivetrain;
    this.subElevator = subElevator;
    this.subAlgaeIntake = subAlgaeIntake;
  }

  @Override
  public void periodic() {
    double elevatorPos, algaeAngle;

    comp0Drivetrain = new Pose3d(subDrivetrain.getPose());

    // they get there instantly.
    if (Robot.isSimulation()) {
      elevatorPos = subElevator.getLastDesiredPosition().in(Units.Meters) / 2;
      algaeAngle = subAlgaeIntake.getLastDesiredPivotAngle().in(Units.Degrees);
    } else {
      // Use real positions
      elevatorPos = (subElevator.getElevatorPosition().in(Units.Meters) / 2);
      algaeAngle = subAlgaeIntake.getPivotAngle().in(Units.Degrees);
    }

    comp1ElevatorStageOne = new Pose3d(new Translation3d(0.0889,
        0,
        0.109474 + elevatorPos), Rotation3d.kZero);

    comp2ElevatorCarriage = comp1ElevatorStageOne
        .transformBy(new Transform3d(new Translation3d(0, 0, Units.Inches.of(1).in(Units.Meters) + elevatorPos),
            Rotation3d.kZero));

    comp3AlgaeIntake = comp2ElevatorCarriage
        .transformBy(new Transform3d(new Translation3d(0.075438, 0, 0.292354), new Rotation3d(0, algaeAngle, 0)));
  }
}
