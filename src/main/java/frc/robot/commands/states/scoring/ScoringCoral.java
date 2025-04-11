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
import frc.robot.Constants.constAlgaeIntake;
import frc.robot.Constants.constCoralOuttake;
import frc.robot.Constants.constElevator;
import frc.robot.Constants.constLED;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.CoralOuttake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.StateMachine.RobotState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoringCoral extends SequentialCommandGroup {
  StateMachine globalStateMachine;
  CoralOuttake globalCoralOuttake;
  Elevator globalElevator;
  LED globalLED;
  SN_XboxController controller;
  RobotState desiredState;
  double coralOuttakeSpeed;
  AlgaeIntake globalAlgaeIntake;

  /** Creates a new CoralScoreSequence. */
  public ScoringCoral(CoralOuttake subCoralOuttake, StateMachine subStateMachine, Elevator globalElevator,
      LED subLED, AlgaeIntake subAlgaeIntake, SN_XboxController controller, RobotState desiredState) {
    globalCoralOuttake = subCoralOuttake;
    globalStateMachine = subStateMachine;
    globalLED = subLED;
    globalAlgaeIntake = subAlgaeIntake;
    this.controller = controller;
    this.desiredState = desiredState;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        // Set state and LEDs
        Commands.runOnce(() -> subStateMachine.setRobotState(StateMachine.RobotState.SCORING_CORAL)),
        Commands.runOnce(() -> globalLED.setLED(constLED.LED_PLACE_CORAL)),
        Commands.runOnce(() -> globalAlgaeIntake.setAlgaeIntakeMotor(constAlgaeIntake.ALGAE_OUTTAKE_NET_SPEED))
            .onlyIf(() -> desiredState.equals(RobotState.PREP_CORAL_L1)),

        // Shoot coral when elevator is at the right position
        Commands.waitUntil(() -> globalElevator.atDesiredPosition()),
        Commands.runOnce(() -> globalCoralOuttake.setCoralOuttakeSpeed(getCoralOuttakeSpeed())),
        Commands.runOnce(() -> globalCoralOuttake.setHasCoral(false)),
        Commands.runOnce(() -> globalElevator.setPosition(constElevator.AFTER_L1_HEIGHT))
            .onlyIf(() -> desiredState.equals(RobotState.PREP_CORAL_L1)),

        // Start ze timer
        Commands.waitSeconds(constCoralOuttake.CORAL_SCORE_TIME.in(Units.Seconds)),
        Commands.waitUntil(() -> !controller.btn_RightTrigger.getAsBoolean()),

        Commands.runOnce(() -> RobotContainer.justScored = true),

        Commands.runOnce(() -> globalAlgaeIntake.setAlgaeIntakeMotor(0))
            .onlyIf(() -> desiredState.equals(RobotState.PREP_CORAL_L1)),

        // Set the state to NONE once the timer is up and the operator lets go of the
        // button
        Commands.deferredProxy(() -> globalStateMachine.tryState(RobotState.NONE)));
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
