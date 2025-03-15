// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.states.prep_coral;

import java.util.function.BooleanSupplier;

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

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PrepCoralLv extends Command {
  StateMachine globalStateMachine;
  Elevator globalElevator;
  AlgaeIntake globalAlgaeIntake;
  Drivetrain globalDrivetrain;
  Distance desiredFaceHeight;
  Distance adjFaceHeight;
  LED globalLED;

  /** Creates a new PrepCoralLv. */
  public PrepCoralLv(StateMachine subStateMachine, Elevator subElevator, AlgaeIntake subAlgaeIntake, Distance height,
      LED subLED, Drivetrain subDrivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    globalStateMachine = subStateMachine;
    this.globalElevator = subElevator;
    this.globalAlgaeIntake = subAlgaeIntake;
    this.desiredFaceHeight = height;
    globalDrivetrain = subDrivetrain;
    globalLED = subLED;
    addRequirements(subElevator);
    addRequirements(globalStateMachine);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (desiredFaceHeight.equals(constElevator.CORAL_L1_HEIGHT))
      globalStateMachine.setRobotState(StateMachine.RobotState.PREP_CORAL_L1);
    else if (desiredFaceHeight.equals(constElevator.CORAL_L2_HEIGHT))
      globalStateMachine.setRobotState(StateMachine.RobotState.PREP_CORAL_L2);
    else if (desiredFaceHeight.equals(constElevator.CORAL_L3_HEIGHT))
      globalStateMachine.setRobotState(StateMachine.RobotState.PREP_CORAL_L3);
    else if (desiredFaceHeight.equals(constElevator.CORAL_L4_HEIGHT))
      globalStateMachine.setRobotState(StateMachine.RobotState.PREP_CORAL_L4);
    globalElevator.setPosition(desiredFaceHeight);
    globalAlgaeIntake.setAlgaePivotAngle(constAlgaeIntake.CORAL_ONLY);
    globalLED.setLED(constLED.LED_PREP_CORAL_LV);
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
