// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.states;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.AlgaeIntake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class EjectingAlgae extends Command {
  /** Creates a new EjectingAlgae. */
  AlgaeIntake globalAlgaeIntake;

  public EjectingAlgae(AlgaeIntake passedAlgaeIntake) {
    // Use addRequirements() here to declare subsystem dependencies.
    globalAlgaeIntake = passedAlgaeIntake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    globalAlgaeIntake.setAlgaeIntakeMotor(Constants.constAlgaeIntake.ALGAE_OUTTAKE_SPEED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    globalAlgaeIntake.setAlgaeIntakeMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}