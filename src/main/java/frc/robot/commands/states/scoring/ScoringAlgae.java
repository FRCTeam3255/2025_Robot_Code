// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.states.scoring;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.constAlgaeIntake;
import frc.robot.Constants.constLED;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.StateMachine;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ScoringAlgae extends Command {
  StateMachine globalStateMachine;
  Elevator globalElevator;
  AlgaeIntake subAlgaeIntake;
  LED globalLED;

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
    globalStateMachine.setRobotState(StateMachine.RobotState.SCORING_ALGAE);
    globalLED.setLED(constLED.LED_SCORING_ALGAE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (globalElevator.isAtSetPoint() && subAlgaeIntake.isAtSetPoint()) {
      subAlgaeIntake.setAlgaeIntakeMotor(constAlgaeIntake.ALGAE_OUTTAKE_SPEED);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subAlgaeIntake.setAlgaeIntakeMotor(0);
    subAlgaeIntake.setHasAlgaeOverride(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
