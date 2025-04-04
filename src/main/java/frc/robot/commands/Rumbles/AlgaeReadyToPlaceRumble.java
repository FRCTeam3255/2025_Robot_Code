// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Rumbles;

import com.frcteam3255.joystick.SN_XboxController;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.constControllers;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.StateMachine;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgaeReadyToPlaceRumble extends Command {
  SN_XboxController globalDriver;
  SN_XboxController globalOperator;
  Elevator globalElevator;
  AlgaeIntake globalAlgaeIntake;
  StateMachine globalStateMachine;

  /** Creates a new ReadyToPlaceCoralRumble. */
  public AlgaeReadyToPlaceRumble(SN_XboxController conDriver, SN_XboxController conOperator, Elevator subElevator,
      AlgaeIntake subAlgaeIntake, StateMachine subStateMachine) {
    globalDriver = conDriver;
    globalOperator = conOperator;
    globalElevator = subElevator;
    globalAlgaeIntake = subAlgaeIntake;
    globalStateMachine = subStateMachine;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    globalDriver.setRumble(RumbleType.kBothRumble, constControllers.READY_TO_PLACE_RUMBLE_INTENSITY);
    globalOperator.setRumble(RumbleType.kBothRumble, constControllers.READY_TO_PLACE_RUMBLE_INTENSITY);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    globalDriver.setRumble(RumbleType.kBothRumble, 0);
    globalOperator.setRumble(RumbleType.kBothRumble, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Stops when ready to shoot anymore, might want to add drivetrain alignment
    // to this
    return (!globalElevator.isAtAnyAlgaeScoringPosition() || !globalAlgaeIntake.isAtAnyAlgaeScoringPosition());
  }
}
