// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.states;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.Constants.constLED;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.StateMachine;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PrepProcessor extends Command {
  /** Creates a new PrepProcessor. */
  StateMachine globalStateMachine;
  Elevator globalElevator;
  AlgaeIntake globalAlgaeIntake;
  LED globalLED;

  public PrepProcessor(StateMachine subStateMachine, Elevator subElevator, AlgaeIntake subAlgaeIntake, LED subLED) {
    // Use addRequirements() here to declare subsystem dependencies.
    globalStateMachine = subStateMachine;
    globalElevator = subElevator;
    globalAlgaeIntake = subAlgaeIntake;
    globalLED = subLED;
    addRequirements(globalStateMachine);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    globalStateMachine.setRobotState(StateMachine.RobotState.PREP_PROCESSOR);
    globalElevator.setPosition(Constants.constElevator.ALGAE_PREP_PROCESSOR_HEIGHT);

    globalAlgaeIntake.setAlgaePivotAngle(Constants.constAlgaeIntake.PREP_PROCESSOR_PIVOT_POSITION);
    globalLED.setLED(constLED.LED_PREP_PROCESSOR);
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
    return globalElevator.isAtSetpoint();
  }
}
