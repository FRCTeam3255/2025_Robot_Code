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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Robot;
import frc.robot.Constants.constAlgaeIntake;
import frc.robot.Constants.constElevator;
import frc.robot.Constants.constField;

@Logged
public class RobotPoses extends SubsystemBase {

  Pose3d comp0Drivetrain = Pose3d.kZero;
  Pose3d comp1ElevatorStageOne = Pose3d.kZero;
  Pose3d comp2ElevatorCarriage = Pose3d.kZero;
  Pose3d comp3AlgaeIntake = Pose3d.kZero;
  Pose3d coralPose = constField.POSES.SCORING_ELEMENT_NOT_COLLECTED;
  Pose3d algaePose = constField.POSES.SCORING_ELEMENT_NOT_COLLECTED;

  @NotLogged
  private Elevator subElevator;
  @NotLogged
  private AlgaeIntake subAlgaeIntake;
  @NotLogged
  private Drivetrain subDrivetrain;
  @NotLogged
  private CoralOuttake subCoralIntake;

  /** Creates a new RobotPoses. */

  public RobotPoses(Drivetrain subDrivetrain, Elevator subElevator, AlgaeIntake subAlgaeIntake,
      CoralOuttake subCoralIntake) {
    this.subDrivetrain = subDrivetrain;
    this.subElevator = subElevator;
    this.subAlgaeIntake = subAlgaeIntake;
    this.subCoralIntake = subCoralIntake;
  }

  @Override
  public void periodic() {
    Distance elevatorPos;
    Angle algaeAngle;

    // -- ROBOT POSITIONS --
    comp0Drivetrain = new Pose3d(subDrivetrain.getPose());

    // If we're in simulation, we can't log real mechanism data because they don't
    // exist. Instead, we'll log where we *want* the mechanisms to be and assume
    // they get there instantly.
    if (Robot.isSimulation()) {
      elevatorPos = subElevator.getLastDesiredPosition().div(2);
      algaeAngle = subAlgaeIntake.getLastDesiredPivotAngle().unaryMinus();
    } else {
      // Use real positions
      elevatorPos = (subElevator.getElevatorPosition().div(2));
      algaeAngle = subAlgaeIntake.getPivotAngle().unaryMinus();
    }

    comp1ElevatorStageOne = new Pose3d(new Translation3d(Units.Meters.of(0.0889),
        Units.Meters.of(0),
        Units.Meters.of(0.109474).plus(elevatorPos)), Rotation3d.kZero);

    comp2ElevatorCarriage = comp1ElevatorStageOne
        .transformBy(new Transform3d(
            new Translation3d(Units.Inches.of(0), Units.Inches.of(0), Units.Inches.of(1).plus(elevatorPos)),
            Rotation3d.kZero));

    comp3AlgaeIntake = comp2ElevatorCarriage
        .transformBy(
            new Transform3d(new Translation3d(Units.Meters.of(0.075438), Units.Meters.of(0), Units.Meters.of(0.292354)),
                new Rotation3d(Units.Degrees.of(0), algaeAngle.plus(constAlgaeIntake.MIN_POS),
                    Units.Degrees.of(0))));

    // -- SCORING ELEMENTS --
    if (subAlgaeIntake.hasAlgae()) {
      algaePose = comp0Drivetrain.plus(new Transform3d(Pose3d.kZero, comp3AlgaeIntake))
          .transformBy(constAlgaeIntake.ALGAE_INTAKE_TO_ALGAE);
    } else {
      algaePose = constField.POSES.SCORING_ELEMENT_NOT_COLLECTED;
    }

    if (subCoralIntake.hasCoral()) {
      coralPose = comp0Drivetrain.plus(new Transform3d(Pose3d.kZero, comp2ElevatorCarriage))
          .transformBy(constElevator.CARRIAGE_TO_CORAL);
    } else {
      coralPose = constField.POSES.SCORING_ELEMENT_NOT_COLLECTED;
    }
  }
}
