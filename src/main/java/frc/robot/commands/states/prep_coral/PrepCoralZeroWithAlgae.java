// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.states.prep_coral;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.constLED;
import frc.robot.subsystems.*;

public class PrepCoralZeroWithAlgae extends Command {
  StateMachine globalStateMachine;
  Elevator globalElevator;
  LED globalLED;
  CoralOuttake globalCoralOuttake;

  public PrepCoralZeroWithAlgae(StateMachine subStateMachine, CoralOuttake subCoralOuttake, Elevator subElevator,
      LED subLED) {
    // Use addRequirements() here to declare subsystem dependencies.
    globalStateMachine = subStateMachine;
    globalElevator = subElevator;
    globalLED = subLED;
    globalCoralOuttake = subCoralOuttake;
    addRequirements(globalStateMachine);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (!globalCoralOuttake.sensorSeesCoral()) {
      globalStateMachine.setRobotState(StateMachine.RobotState.PREP_CORAL_ZERO_WITH_ALGAE);
      globalElevator.setPosition(Constants.constElevator.PREP_0);
      globalLED.setLED(constLED.LED_PREP_CORAL_ZERO_WITH_ALGAE);
    } else {
      System.out.println("PrepCoralLv: Coral is in the way");
    }
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
    return globalElevator.atDesiredPosition();
  }
}
