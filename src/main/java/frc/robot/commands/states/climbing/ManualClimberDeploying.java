// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.states.climbing;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.*;
import frc.robot.Elastic;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.StateMachine.RobotState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ManualClimberDeploying extends Command {
  /** Creates a new ManualClimberDeploying. */
  StateMachine globalStateMachine;
  Climber globalClimber;
  LED globalLED;
  AlgaeIntake globalAlgaeIntake;
  Elevator globalElevator;

  public ManualClimberDeploying(StateMachine subStateMachine, Climber subClimber, Elevator subElevator,
      AlgaeIntake subAlgaeIntake, LED subLED) {
    // Use addRequirements() here to declare subsystem dependencies.
    globalStateMachine = subStateMachine;
    globalClimber = subClimber;
    globalElevator = subElevator;
    globalLED = subLED;
    globalAlgaeIntake = subAlgaeIntake;

    addRequirements(globalStateMachine);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Elastic.selectTab("Climbing");
    globalStateMachine.setRobotState(RobotState.MANUAL_CLIMBER_DEPLOYING);
    globalAlgaeIntake.setAlgaePivotAngle(Constants.constAlgaeIntake.CLIMB_DEPLOY_POSITION);
    globalElevator.setPosition(Constants.constElevator.ZEROED_POS);
    globalClimber.setClimberMotorVelocity(Constants.constClimber.MANUAL_CLIMBER_MOTOR_DEPLOYING_VELOCITY);
    globalLED.setLED(constLED.LED_MANUAL_CLIMBER_DEPLOYING);
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
