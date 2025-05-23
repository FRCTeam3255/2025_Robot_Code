// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.states.first_scoring_element;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.constLED;
import frc.robot.subsystems.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeCoralHopper extends Command {
  StateMachine globalStateMachine;
  Hopper globalHopper;
  CoralOuttake globalCoralOuttake;
  Elevator globalElevator;
  LED globalLED;
  AlgaeIntake globalAlgaeIntake;

  /** Creates a new IntakeCoralHopper. */
  public IntakeCoralHopper(StateMachine subStateMachine, Hopper subHopper, CoralOuttake subCoralOuttake,
      LED subLED, Elevator subElevator, AlgaeIntake subAlgaeIntake) {
    // Use addRequirements() here to declare subsystem dependencies.
    globalStateMachine = subStateMachine;
    this.globalHopper = subHopper;
    this.globalCoralOuttake = subCoralOuttake;
    globalLED = subLED;
    globalElevator = subElevator;
    globalAlgaeIntake = subAlgaeIntake;
    addRequirements(globalStateMachine);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    globalStateMachine.setRobotState(StateMachine.RobotState.INTAKING_CORAL);
    globalAlgaeIntake.setAlgaePivotAngle(Constants.constAlgaeIntake.CORAL_ONLY);
    globalCoralOuttake.setCoralOuttakeSpeed(Constants.constCoralOuttake.CORAL_INTAKE_SPEED);
    globalHopper.runHopper(Constants.constHopper.HOPPER_SPEED);
    globalLED.setLED(constLED.LED_INTAKE_CORAL_HOPPER);
    globalElevator.setPosition(Constants.constElevator.CORAL_INTAKE_HIGHT);
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
    return globalCoralOuttake.sensorSeesCoral();

  }
}