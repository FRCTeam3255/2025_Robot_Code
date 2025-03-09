// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.states.prep_algae;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.StateMachine.RobotState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PrepNet extends SequentialCommandGroup {
  StateMachine globalStateMachine;
  Elevator globalElevator;
  AlgaeIntake globalAlgaeIntake;
  LED globalLED;

  /** Creates a new PrepNet1. */
  public PrepNet(StateMachine subStateMachine, Elevator subElevator, AlgaeIntake subAlgaeIntake, LED subLED) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    globalStateMachine = subStateMachine;
    globalElevator = subElevator;
    globalAlgaeIntake = subAlgaeIntake;
    globalLED = subLED;
    addRequirements(globalStateMachine);
    addCommands(
        Commands.runOnce(() -> subStateMachine.setRobotState(RobotState.PREP_NET)),
        Commands.runOnce(
            () -> subAlgaeIntake.setAlgaePivotAngle(Constants.constAlgaeIntake.SAFE_TO_MOVE_ELEVATOR_PIVOT_POSITION)),
        Commands.runOnce(() -> globalLED.setLED(Constants.constLED.LED_PREP_NET)),
        Commands.waitUntil(() -> globalAlgaeIntake.isAtSetPoint()),
        Commands.runOnce(() -> globalElevator.setPosition(Constants.constElevator.ALGAE_PREP_NET)),
        Commands.waitUntil(() -> globalElevator.isAtSetPoint()),
        Commands
            .runOnce(() -> globalAlgaeIntake.setAlgaePivotAngle(Constants.constAlgaeIntake.PREP_NET_PIVOT_POSITION)));
  }
}
