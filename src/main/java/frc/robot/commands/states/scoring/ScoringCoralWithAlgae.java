// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.states.scoring;

import com.frcteam3255.joystick.SN_XboxController;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.constCoralOuttake;
import frc.robot.Constants.constLED;
import frc.robot.subsystems.CoralOuttake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.StateMachine.RobotState;

public class ScoringCoralWithAlgae extends SequentialCommandGroup {
  StateMachine globalStateMachine;
  CoralOuttake globalCoralOuttake;
  Elevator globalElevator;
  LED globalLED;
  SN_XboxController controller;
  RobotState desiredState;
  double coralOuttakeSpeed;

  public ScoringCoralWithAlgae(CoralOuttake subCoralOuttake, StateMachine subStateMachine, Elevator globalElevator,
      LED subLED, SN_XboxController controller, RobotState desiredState) {
    globalCoralOuttake = subCoralOuttake;
    globalStateMachine = subStateMachine;
    globalLED = subLED;
    this.controller = controller;
    this.desiredState = desiredState;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        // Set state and LEDs
        Commands.runOnce(() -> subStateMachine.setRobotState(StateMachine.RobotState.SCORING_CORAL_WITH_ALGAE)),
        Commands.runOnce(() -> globalLED.setLED(constLED.LED_PLACE_CORAL_WITH_ALGAE)),

        // Shoot coral when elevator is at the right position
        Commands.waitUntil(() -> globalElevator.atDesiredPosition()),
        Commands.runOnce(() -> globalCoralOuttake.setCoralOuttakeSpeed(getCoralOuttakeSpeed())),
        Commands.runOnce(() -> globalCoralOuttake.setHasCoral(false)),

        // Start ze timer
        Commands.waitSeconds(constCoralOuttake.CORAL_SCORE_TIME.in(Units.Seconds)),
        Commands.waitUntil(() -> !controller.btn_RightTrigger.getAsBoolean()),

        Commands.runOnce(() -> RobotContainer.justScored = true),

        // Set the state to NONE once the timer is up and the operator lets go of the
        // button
        Commands.deferredProxy(() -> globalStateMachine.tryState(RobotState.HAS_ALGAE)));
  }

  public double getCoralOuttakeSpeed() {
    if (desiredState.equals(RobotState.PREP_CORAL_L4)) {
      coralOuttakeSpeed = Constants.constCoralOuttake.CORAL_L4_OUTTAKE_SPEED;
    } else if (desiredState.equals(RobotState.PREP_CORAL_L1)) {
      coralOuttakeSpeed = Constants.constCoralOuttake.CORAL_L1_OUTTAKE_SPEED;
    } else {
      coralOuttakeSpeed = Constants.constCoralOuttake.CORAL_OUTTAKE_SPEED;
    }

    return coralOuttakeSpeed;
  }
}
