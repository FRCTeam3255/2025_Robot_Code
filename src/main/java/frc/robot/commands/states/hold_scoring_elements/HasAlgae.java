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

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HasAlgae extends Command {
  StateMachine globalStateMachine;
  AlgaeIntake globalAlgaeIntake;
  LED globalLED;
  CoralOuttake globalCoralOuttake;
  Hopper globalHopper;
  Elevator globalElevator;

  /** Creates a new HasAlgae. */
  public HasAlgae(StateMachine subStateMachine, AlgaeIntake subAlgaeIntake, LED subLED, CoralOuttake subCoralOuttake,
      Hopper subHopper, Elevator subElevator) {
    globalStateMachine = subStateMachine;
    globalAlgaeIntake = subAlgaeIntake;
    globalLED = subLED;
    globalCoralOuttake = subCoralOuttake;
    globalHopper = subHopper;
    globalElevator = subElevator;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(globalStateMachine);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (globalStateMachine.getRobotState() == RobotState.SCORING_CORAL_WITH_ALGAE) {
      globalCoralOuttake.setCoralOuttake(0);
      globalHopper.runHopper(0);
      globalAlgaeIntake.setAlgaePivotAngle(constAlgaeIntake.PREP_ALGAE_ZERO_PIVOT_POSITION);
      globalElevator.setPosition(Units.Inches.zero());
    }
    globalStateMachine.setRobotState(RobotState.HAS_ALGAE);
    globalAlgaeIntake.setHasAlgaeOverride(true);
    globalLED.setLEDMatrix(constLED.LED_HAS_ALGAE, 0, 4);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    globalAlgaeIntake.setAlgaeIntakeVoltage(constAlgaeIntake.HOLD_ALGAE_INTAKE_VOLTAGE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
