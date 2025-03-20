// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.states.prep_coral;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.constAlgaeIntake;
import frc.robot.Constants.constDrivetrain;
import frc.robot.Constants.constElevator;
import frc.robot.Constants.constLED;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.StateMachine;

public class PrepCoralLvWithAlgae extends Command {
  StateMachine globalStateMachine;
  Elevator globalElevator;
  Drivetrain globalDrivetrain;
  AlgaeIntake globalAlgaeIntake;
  Distance desiredFaceHeight;
  Distance adjFaceHeight;
  LED globalLED;

  public PrepCoralLvWithAlgae(StateMachine subStateMachine, Elevator subElevator, Distance height, LED subLED,
      AlgaeIntake subAlgaeIntake, Drivetrain subDrivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    globalStateMachine = subStateMachine;
    this.globalElevator = subElevator;
    this.desiredFaceHeight = height;
    this.globalAlgaeIntake = subAlgaeIntake;
    globalDrivetrain = subDrivetrain;
    globalLED = subLED;
    addRequirements(subElevator);
    addRequirements(globalStateMachine);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    globalAlgaeIntake.setAlgaePivotAngle(constAlgaeIntake.PREP_ALGAE_ZERO_PIVOT_POSITION);
    if (desiredFaceHeight.equals(constElevator.CORAL_L1_HEIGHT))
      globalStateMachine.setRobotState(StateMachine.RobotState.PREP_CORAL_L1_WITH_ALGAE);
    else if (desiredFaceHeight.equals(constElevator.CORAL_L2_HEIGHT))
      globalStateMachine.setRobotState(StateMachine.RobotState.PREP_CORAL_L2_WITH_ALGAE);
    else if (desiredFaceHeight.equals(constElevator.CORAL_L3_HEIGHT))
      globalStateMachine.setRobotState(StateMachine.RobotState.PREP_CORAL_L3_WITH_ALGAE);
    else if (desiredFaceHeight.equals(constElevator.CORAL_L4_HEIGHT))
      globalStateMachine.setRobotState(StateMachine.RobotState.PREP_CORAL_L4_WITH_ALGAE);
    globalElevator.setPosition(desiredFaceHeight);
    globalLED.setLED(constLED.LED_PREP_CORAL_LV_WITH_ALGAE);
    adjFaceHeight = desiredFaceHeight.plus(constElevator.HEIGHT_SHIFT);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (globalDrivetrain.atPose(globalDrivetrain.desiredAlignmentPose, constDrivetrain.SHIFT_ELEVATOR_TOLERANCE,
        constDrivetrain.TELEOP_AUTO_ALIGN.AT_ROTATION_TOLERANCE) && globalStateMachine.isAutoDriving()) {
      globalElevator.setPosition(adjFaceHeight);
    } else {
      globalElevator.setPosition(desiredFaceHeight);
    }

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
