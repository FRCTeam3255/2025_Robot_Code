// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.states;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.constLED;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.StateMachine.RobotState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakingAlgaeGround extends Command {
  /** Creates a new IntakingAlgaeGround. */
  StateMachine subStateMachine;
  Elevator globalElevator;
  AlgaeIntake globalAlgaeIntake;
  LED subLED;

  public IntakingAlgaeGround(StateMachine subStateMachine, Elevator subElevator, AlgaeIntake subAlgaeIntake,
      LED globalLED) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.subStateMachine = subStateMachine;
    globalElevator = subElevator;
    globalAlgaeIntake = subAlgaeIntake;
    subLED = globalLED;
    addRequirements(subStateMachine);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    subStateMachine.setRobotState(RobotState.INTAKING_ALGAE_GROUND);
    globalElevator.setPosition(Constants.constElevator.ALGAE_GROUND_INTAKE);
    globalAlgaeIntake.setAlgaeIntakeMotor(Constants.constAlgaeIntake.ALGAE_INTAKE_SPEED);
    subLED.setLED(constLED.LED_INTAKE_ALGAE_GROUND);
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
    return globalAlgaeIntake.hasAlgae();
  }
}
