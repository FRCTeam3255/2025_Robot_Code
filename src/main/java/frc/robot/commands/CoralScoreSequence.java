// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.constCoralOuttake;
import frc.robot.subsystems.CoralOuttake;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.StateMachine.RobotState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CoralScoreSequence extends SequentialCommandGroup {
  double startTime = 0;
  StateMachine globalStateMachine;
  CoralOuttake globalCoralOuttake;

  /** Creates a new CoralScoreSequence. */
  public CoralScoreSequence(CoralOuttake subCoralOuttake, StateMachine subStateMachine) {
    globalCoralOuttake = subCoralOuttake;
    globalStateMachine = subStateMachine;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

        Commands.runOnce(() -> startTime = Timer.getFPGATimestamp()),
        Commands.waitUntil(() -> !globalCoralOuttake.hasCoral()),
        Commands.waitTime(constCoralOuttake.CORAL_SCORE_TIME),
        Commands.runOnce(() -> globalStateMachine.setRobotState(RobotState.NONE)));
  }
}
