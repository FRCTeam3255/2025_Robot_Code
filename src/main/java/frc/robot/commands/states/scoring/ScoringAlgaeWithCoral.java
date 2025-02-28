// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.states.scoring;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.constAlgaeIntake;
import frc.robot.Constants.constLED;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.StateMachine.RobotState;

public class ScoringAlgaeWithCoral extends Command {
  StateMachine globalStateMachine;
  Elevator globalElevator;
  AlgaeIntake subAlgaeIntake;
  LED globalLED;
  double desiredSpeed;

  public ScoringAlgaeWithCoral(StateMachine subStateMachine, AlgaeIntake subAlgaeIntake, LED subLED,
      Elevator subElevator) {
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
    if (globalStateMachine.getRobotState() == RobotState.PREP_NET_WITH_CORAL) {
      desiredSpeed = constAlgaeIntake.ALGAE_OUTTAKE_NET_SPEED;
    } else {
      desiredSpeed = constAlgaeIntake.ALGAE_OUTTAKE_PROCESSOR_SPEED;
    }

    globalStateMachine.setRobotState(StateMachine.RobotState.SCORING_ALGAE_WITH_CORAL);
    globalLED.setLED(constLED.LED_SCORING_ALGAE_WITH_CORAL);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (globalElevator.isAtSetPoint() && subAlgaeIntake.isAtSetPoint()) {
      subAlgaeIntake.setAlgaeIntakeMotor(desiredSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subAlgaeIntake.setAlgaeIntakeMotor(0);
    subAlgaeIntake.setHasAlgaeOverride(false);
    RobotContainer.justScored = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
