// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.states.prep_coral;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.constAlgaeIntake;
import frc.robot.Constants.constElevator;
import frc.robot.Constants.constLED;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.CoralOuttake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.StateMachine;

public class PrepCoralLvWithAlgae extends Command {
  StateMachine globalStateMachine;
  Elevator globalElevator;
  Distance globalDistance;
  AlgaeIntake globalAlgaeIntake;
  LED globalLED;
  CoralOuttake globalCoralOuttake;

  public PrepCoralLvWithAlgae(StateMachine subStateMachine, CoralOuttake subCoralOuttake, Elevator subElevator,
      Distance height, LED subLED,
      AlgaeIntake subAlgaeIntake) {
    // Use addRequirements() here to declare subsystem dependencies.
    globalStateMachine = subStateMachine;
    this.globalElevator = subElevator;
    this.globalDistance = height;
    this.globalAlgaeIntake = subAlgaeIntake;
    globalLED = subLED;
    globalCoralOuttake = subCoralOuttake;
    addRequirements(subElevator);
    addRequirements(globalStateMachine);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (!globalCoralOuttake.sensorSeesCoral()) {
      globalAlgaeIntake.setAlgaePivotAngle(constAlgaeIntake.PREP_ALGAE_ZERO_PIVOT_POSITION);
      if (globalDistance.equals(constElevator.CORAL_L1_HEIGHT))
        globalStateMachine.setRobotState(StateMachine.RobotState.PREP_CORAL_L1_WITH_ALGAE);
      else if (globalDistance.equals(constElevator.CORAL_L2_HEIGHT))
        globalStateMachine.setRobotState(StateMachine.RobotState.PREP_CORAL_L2_WITH_ALGAE);
      else if (globalDistance.equals(constElevator.CORAL_L3_HEIGHT))
        globalStateMachine.setRobotState(StateMachine.RobotState.PREP_CORAL_L3_WITH_ALGAE);
      else if (globalDistance.equals(constElevator.CORAL_L4_HEIGHT))
        globalStateMachine.setRobotState(StateMachine.RobotState.PREP_CORAL_L4_WITH_ALGAE);
      globalElevator.setPosition(globalDistance);
      globalLED.setLED(constLED.LED_PREP_CORAL_LV_WITH_ALGAE);
    }
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
    return globalElevator.atDesiredPosition();
  }
}
