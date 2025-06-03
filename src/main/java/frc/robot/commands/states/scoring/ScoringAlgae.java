// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.states.scoring;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.constAlgaeIntake;
import frc.robot.Constants.constElevator;
import frc.robot.Constants.constLED;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.StateMachine.RobotState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ScoringAlgae extends Command {
  StateMachine globalStateMachine;
  Elevator globalElevator;
  AlgaeIntake subAlgaeIntake;
  LED globalLED;
  double desiredSpeed;
  Distance desiredSetpoint;
  Distance elevatorTolerance = constElevator.DEADZONE_DISTANCE;
  boolean ignoreAlgaePivot;

  public ScoringAlgae(StateMachine subStateMachine, AlgaeIntake subAlgaeIntake, LED subLED, Elevator subElevator) {
    // Use addRequirements() here to declare subsystem dependencies.
    globalStateMachine = subStateMachine;
    this.subAlgaeIntake = subAlgaeIntake;
    globalElevator = subElevator;
    globalLED = subLED;
    addRequirements(globalStateMachine);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (globalStateMachine.getRobotState() == RobotState.PREP_NET) {
      desiredSetpoint = constElevator.ALGAE_PREP_NET;
      desiredSpeed = (edu.wpi.first.wpilibj.RobotState.isAutonomous()) ? -1
          : constAlgaeIntake.ALGAE_OUTTAKE_NET_SPEED;
      elevatorTolerance = constElevator.NET_TOLERANCE;
    } else if (globalStateMachine.getRobotState() == RobotState.PREP_PROCESSOR) {
      desiredSetpoint = constElevator.ALGAE_PREP_PROCESSOR_HEIGHT;
      desiredSpeed = constAlgaeIntake.ALGAE_OUTTAKE_PROCESSOR_SPEED;
    } else {
      desiredSetpoint = constElevator.PREP_0;
      desiredSpeed = constAlgaeIntake.ALGAE_OUTTAKE_EJECT_SPEED;
    }

    ignoreAlgaePivot = edu.wpi.first.wpilibj.RobotState.isTeleop();
    globalStateMachine.setRobotState(StateMachine.RobotState.SCORING_ALGAE);
    globalLED.setLED(constLED.LED_SCORING_ALGAE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (ignoreAlgaePivot) {
      if (globalElevator.isAtSetPointWithTolerance(desiredSetpoint, elevatorTolerance)) {
        subAlgaeIntake.setAlgaeIntakeMotor(desiredSpeed);
      }
    } else {
      if (globalElevator.isAtSetPointWithTolerance(desiredSetpoint, elevatorTolerance)
          && subAlgaeIntake.isAtSetPoint()) {
        subAlgaeIntake.setAlgaeIntakeMotor(desiredSpeed);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subAlgaeIntake.setAlgaeIntakeMotor(0);
    subAlgaeIntake.setHasAlgaeOverride(false);
    RobotContainer.justScored = true;
    subAlgaeIntake.YEET = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
