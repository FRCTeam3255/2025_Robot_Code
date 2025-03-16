// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.states.second_scoring_element;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.constAlgaeIntake;
import frc.robot.Constants.constLED;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.StateMachine.RobotState;

public class CleaningL2ReefWithCoral extends Command {
  StateMachine globalStateMachine;
  Elevator globalElevator;
  AlgaeIntake globalAlgaeIntake;
  LED globalLED;

  public CleaningL2ReefWithCoral(StateMachine subStateMachine, Elevator subElevator, AlgaeIntake subAlgaeIntake,
      LED subLED) {
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
    globalAlgaeIntake.setHasAlgaeOverride(false);
    globalStateMachine.setRobotState(RobotState.CLEANING_L2_WITH_CORAL);
    globalElevator.setPosition(Constants.constElevator.ALGAE_L2_CLEANING);
    globalAlgaeIntake.setAlgaeIntakeMotor(Constants.constAlgaeIntake.ALGAE_INTAKE_SPEED);
    globalLED.setLED(constLED.LED_CLEANING_L2_REEF);

    globalAlgaeIntake.setAlgaePivotAngle(Constants.constAlgaeIntake.CLEANING_REEF_L2_PIVOT_POSITION);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!globalAlgaeIntake.hasAlgae()) {
      globalAlgaeIntake.setAlgaePivotAngle(constAlgaeIntake.PREP_ALGAE_ZERO_PIVOT_POSITION);
      globalElevator.setPosition(Constants.constElevator.PREP_0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
