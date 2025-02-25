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
import frc.robot.subsystems.LED;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.StateMachine.RobotState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HasCoral extends Command {
  /** Creates a new HasCoral. */
  StateMachine globalStateMachine;
  CoralOuttake globalCoralOuttake;
  AlgaeIntake globalAlgaeIntake;
  Elevator globalElevator;
  LED globalLED;

  public HasCoral(StateMachine subStateMachine, CoralOuttake subCoralOuttake, LED subLED, AlgaeIntake subAlgaeIntake,
      Elevator subElevator) {
    // Use addRequirements() here to declare subsystem dependencies.
    globalStateMachine = subStateMachine;
    globalCoralOuttake = subCoralOuttake;
    globalAlgaeIntake = subAlgaeIntake;
    globalElevator = subElevator;
    globalLED = subLED;
    addRequirements(globalStateMachine);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    globalStateMachine.setRobotState(RobotState.HAS_CORAL);
    globalAlgaeIntake.setAlgaePivotAngle(constAlgaeIntake.PREP_ALGAE_ZERO_PIVOT_POSITION);
    globalAlgaeIntake.setAlgaeIntakeMotor(0);
    globalElevator.setPosition(Units.Inches.zero());
    globalLED.setLED(constLED.LED_HAS_CORAL);
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