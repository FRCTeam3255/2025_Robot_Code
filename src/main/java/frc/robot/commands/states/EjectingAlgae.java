// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.states;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.constLED;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.StateMachine.RobotState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class EjectingAlgae extends Command {
  /** Creates a new EjectingAlgae. */
  AlgaeIntake globalAlgaeIntake;
  StateMachine globalStateMachine;
  LED globalLED;

  public EjectingAlgae(StateMachine subStateMachine, AlgaeIntake subAlgaeIntake, LED subLED) {
    // Use addRequirements() here to declare subsystem dependencies.
    globalStateMachine = subStateMachine;
    globalAlgaeIntake = subAlgaeIntake;
    globalLED = subLED;
    addRequirements(globalStateMachine);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    globalStateMachine.setRobotState(RobotState.EJECTING_ALGAE);
    globalAlgaeIntake.setAlgaeIntakeMotor(Constants.constAlgaeIntake.ALGAE_OUTTAKE_SPEED);
    globalLED.setLED(constLED.LED_EJECTING_ALGAE);

    globalAlgaeIntake.setAlgaePivotAngle(Constants.constAlgaeIntake.EJECT_ALGAE_PIVOT_POSITION);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    globalAlgaeIntake.setAlgaeIntakeMotor(0);
    globalAlgaeIntake.setHasAlgaeOverride(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
