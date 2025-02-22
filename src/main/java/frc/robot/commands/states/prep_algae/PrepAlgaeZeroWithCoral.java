// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.states.prep_algae;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.constLED;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.StateMachine;

public class PrepAlgaeZeroWithCoral extends Command {
  StateMachine globalStateMachine;
  Elevator globalElevator;
  LED globalLED;
  AlgaeIntake globalAlgaeIntake;

  public PrepAlgaeZeroWithCoral(StateMachine subStateMachine, Elevator subElevator, AlgaeIntake subAlgaeIntake,
      LED subLED) {
    // Use addRequirements() here to declare subsystem dependencies.
    globalStateMachine = subStateMachine;
    globalElevator = subElevator;
    globalLED = subLED;
    globalAlgaeIntake = subAlgaeIntake;

    addRequirements(globalStateMachine);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    globalStateMachine.setRobotState(StateMachine.RobotState.PREP_ALGAE_ZERO);
    globalElevator.setPosition(Constants.constElevator.PREP_0);
    globalLED.setLED(constLED.LED_PREP_ALGAE_ZERO);

    globalAlgaeIntake.setAlgaePivotAngle(Constants.constAlgaeIntake.PREP_ALGAE_ZERO_PIVOT_POSITION);
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
    return globalElevator.isAtSetPoint();
  }
}
