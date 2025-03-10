// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.states.first_scoring_element;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.constLED;
import frc.robot.subsystems.CoralOuttake;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.StateMachine.RobotState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class EjectCoral extends Command {
  StateMachine globalStateMachine;
  CoralOuttake globalCoralOuttake;
  LED globalLED;
  Hopper globalHopper;

  /** Creates a new CoralOuttake. */
  public EjectCoral(StateMachine subStateMachine, CoralOuttake subCoralOuttake, LED subLED, Hopper subHopper) {
    // Use addRequirements() here to declare subsystem dependencies.
    globalStateMachine = subStateMachine;
    globalCoralOuttake = subCoralOuttake;
    globalLED = subLED;
    globalHopper = subHopper;
    addRequirements(globalStateMachine);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    globalStateMachine.setRobotState(RobotState.EJECTING_CORAL);
    globalCoralOuttake.setCoralOuttake(Constants.constCoralOuttake.CORAL_REVERSE_OUTTAKE_SPEED);
    globalHopper.runHopper(Constants.constHopper.HOPPER_EJECTING_SPEED);
    globalLED.setLED(constLED.LED_EJECT_CORAL);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    globalCoralOuttake.setCoralOuttake(0);
    globalCoralOuttake.setHasCoral(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}