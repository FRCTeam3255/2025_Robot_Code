// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.states;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.*;
import frc.robot.subsystems.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class None extends Command {
  StateMachine globalStateMachine;
  CoralOuttake subCoralOuttake;
  Hopper subHopper;
  AlgaeIntake subAlgaeIntake;
  Climber subClimber;
  Elevator subElevator;
  LED globalLED;

  /** Creates a new none. */
  public None(StateMachine subStateMachine, CoralOuttake subCoralOuttake, Hopper subHopper,
      AlgaeIntake subAlgaeIntake, Climber subClimber, Elevator subElevator, LED subLED) {
    // Use addRequirements() here to declare subsystem dependencies.
    globalStateMachine = subStateMachine;
    this.subCoralOuttake = subCoralOuttake;
    this.subHopper = subHopper;
    this.subAlgaeIntake = subAlgaeIntake;
    this.subClimber = subClimber;
    this.subElevator = subElevator;
    globalLED = subLED;
    addRequirements(globalStateMachine);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    globalStateMachine.setRobotState(StateMachine.RobotState.NONE);
    subCoralOuttake.setCoralOuttakeSpeed(0);
    subHopper.runHopper(0);
    subAlgaeIntake.setAlgaeIntakeMotor(0);
    subAlgaeIntake.setAlgaePivotAngle(constAlgaeIntake.CORAL_ONLY);
    subClimber.setClimberMotorVelocity(0);
    subElevator.setPosition(Units.Inches.zero());
    globalLED.setLED(constLED.LED_NONE);
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
    return false;
  }
}
