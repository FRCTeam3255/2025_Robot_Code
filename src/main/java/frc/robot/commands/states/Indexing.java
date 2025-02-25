package frc.robot.commands.states;

import com.frcteam3255.joystick.SN_XboxController;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.constCoralOuttake;
import frc.robot.subsystems.CoralOuttake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.StateMachine.RobotState;

public class Indexing extends SequentialCommandGroup {
  StateMachine globalStateMachine;
  CoralOuttake globalCoralOuttake;
  Elevator globalElevator;
  SN_XboxController controller;
  RobotState desiredState;
  double coralOuttakeSpeed;

  public Indexing(StateMachine subStateMachine, CoralOuttake subCoralOuttake, Elevator subElevator, SN_XboxController controller, RobotState desiredState) {
    // Use addRequirements() here to declare subsystem dependencies.
    globalStateMachine = subStateMachine;
    globalCoralOuttake = subCoralOuttake;
    globalElevator = subElevator;
    this.controller = controller;

    addRequirements(globalStateMachine, globalElevator);

    addCommands(
    // Shoot coral when elevator is at the right position
    Commands.waitUntil(() -> globalElevator.isAtSetPoint()),
    Commands.runOnce(() -> globalCoralOuttake.setCoralOuttake(getCoralOuttakeSpeed())),
    Commands.runOnce(() -> globalCoralOuttake.setHasCoral(false)),

    // Start ze timer
    Commands.waitSeconds(constCoralOuttake.CORAL_SCORE_TIME.in(Units.Seconds)),
    Commands.waitUntil(() -> !controller.btn_RightTrigger.getAsBoolean()),

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