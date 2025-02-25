package frc.robot.commands.states;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CoralOuttake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.StateMachine.RobotState;

public class Indexing extends Command {
  StateMachine globalStateMachine;
  CoralOuttake globalCoralOuttake;
  Elevator globalElevator;

  public Indexing(StateMachine subStateMachine, CoralOuttake subCoralOuttake, Elevator subElevator) {
    // Use addRequirements() here to declare subsystem dependencies.
    globalStateMachine = subStateMachine;
    globalCoralOuttake = subCoralOuttake;
    globalElevator = subElevator;
    addRequirements(globalStateMachine, globalElevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    globalStateMachine.setRobotState(StateMachine.RobotState.NONE);
    addCommands(
      // Set state and LEDs

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

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
