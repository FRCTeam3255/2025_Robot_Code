// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.states.first_scoring_element;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.CoralOuttake;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.StateMachine.RobotState;

public class IndexingCoral extends Command {
  StateMachine globalStateMachine;
  Hopper globalHopper;
  CoralOuttake globalCoralOuttake;
  AlgaeIntake globalAlgaeIntake;

  public IndexingCoral(StateMachine subStateMachine, Hopper subHopper, CoralOuttake subCoralOuttake,
      AlgaeIntake subAlgaeIntake) {
    globalStateMachine = subStateMachine;
    globalHopper = subHopper;
    globalCoralOuttake = subCoralOuttake;
    globalAlgaeIntake = subAlgaeIntake;
    addRequirements(globalStateMachine);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Begin indexing :)
    globalStateMachine.setRobotState(RobotState.INDEXING_CORAL);
    globalCoralOuttake.setIndexingCoral(true);
    globalCoralOuttake.setCoralOuttakeSpeed(Constants.constCoralOuttake.CORAL_INDEXING_SPEED);
    globalHopper.runHopper(Constants.constHopper.HOPPER_INDEXING_SPEED);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // stop indexing
    globalCoralOuttake.setIndexingCoral(false);
    globalCoralOuttake.setHasCoral(true);
    globalCoralOuttake.setCoralOuttakeSpeed(0);
    globalHopper.runHopper(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return globalCoralOuttake.sensorIndexedCoral();
  }
}
