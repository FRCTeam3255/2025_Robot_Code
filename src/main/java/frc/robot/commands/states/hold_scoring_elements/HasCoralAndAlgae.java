// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.states.hold_scoring_elements;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.constAlgaeIntake;
import frc.robot.Constants.constLED;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.CoralOuttake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.StateMachine.RobotState;

public class HasCoralAndAlgae extends Command {
  /** Creates a new HasCoral. */
  StateMachine globalStateMachine;
  CoralOuttake globalCoralOuttake;
  LED globalLED;
  AlgaeIntake globalAlgaeIntake;
  Elevator globalElevator;
  Hopper globalHopper;

  public HasCoralAndAlgae(StateMachine subStateMachine, CoralOuttake subCoralOuttake, LED subLED,
      AlgaeIntake subAlgaeIntake,
      Elevator subElevator, Hopper subHopper) {
    // Use addRequirements() here to declare subsystem dependencies.
    globalStateMachine = subStateMachine;
    globalCoralOuttake = subCoralOuttake;
    globalAlgaeIntake = subAlgaeIntake;
    globalHopper = subHopper;
    globalElevator = subElevator;
    globalLED = subLED;
    addRequirements(globalStateMachine);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    globalStateMachine.setRobotState(RobotState.HAS_CORAL_AND_ALGAE);
    globalAlgaeIntake.setAlgaePivotAngle(constAlgaeIntake.PREP_ALGAE_ZERO_PIVOT_POSITION);
    globalAlgaeIntake.setAlgaeIntakeVoltage(constAlgaeIntake.HOLD_ALGAE_INTAKE_VOLTAGE);
    globalLED.setLED(constLED.LED_HAS_BOTH);
    globalCoralOuttake.setCoralOuttake(0);
    globalHopper.runHopper(0);

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
    return true;
  }
}