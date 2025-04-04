// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.states.climbing;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Elastic;
import frc.robot.Constants.constLED;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.StateMachine;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimberRetracting extends Command {
  /** Creates a new ClimberTester. */
  Climber globalClimber;
  StateMachine globalStateMachine;
  AlgaeIntake globalAlgaeIntake;
  LED globalLED;

  public ClimberRetracting(StateMachine subStateMachine, Climber subClimber, AlgaeIntake subAlgaeIntake, LED subLED) {
    // Use addRequirements() here to declare subsystem dependencies.
    globalStateMachine = subStateMachine;
    globalClimber = subClimber;
    globalAlgaeIntake = subAlgaeIntake;
    globalLED = subLED;

    addRequirements(globalStateMachine);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Elastic.selectTab("Climbing");
    globalClimber.setClimberMotorVelocity(Constants.constClimber.CLIMBER_RETRACT_VELOCITY);
    globalStateMachine.setRobotState(StateMachine.RobotState.CLIMBER_RETRACTING);
    globalLED.setLED(constLED.LED_CLIMBER_RETRACTING);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    globalClimber.setClimberMotorVelocity(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
