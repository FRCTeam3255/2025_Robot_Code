// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.frcteam3255.joystick.SN_XboxController;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.constCoralOuttake;
import frc.robot.commands.states.PlaceCoral;
import frc.robot.subsystems.CoralOuttake;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.StateMachine.RobotState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CoralScoreSequence extends SequentialCommandGroup {
  StateMachine globalStateMachine;
  CoralOuttake globalCoralOuttake;
  SN_XboxController controller;
  PlaceCoral globalPlaceCoral;

  /** Creates a new CoralScoreSequence. */
  public CoralScoreSequence(CoralOuttake subCoralOuttake, StateMachine subStateMachine, SN_XboxController controller,
      PlaceCoral comPlaceCoral) {
    globalCoralOuttake = subCoralOuttake;
    globalStateMachine = subStateMachine;
    this.controller = controller;
    globalPlaceCoral = comPlaceCoral;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        Commands.runOnce(() -> globalCoralOuttake.setCoralOuttake(comPlaceCoral.getCoralOuttakeSpeed())),
        Commands.waitSeconds(constCoralOuttake.CORAL_SCORE_TIME.in(Units.Seconds)),
        Commands.waitUntil(() -> !controller.btn_RightTrigger.getAsBoolean()),
        Commands.deferredProxy(() -> globalStateMachine.tryState(RobotState.NONE)));
  }
}
