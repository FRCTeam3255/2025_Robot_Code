// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.states;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CoralOuttake;

import frc.robot.subsystems.Hopper;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class None extends Command {
  CoralOuttake subCoralOuttake;
  Hopper subHopper;
  AlgaeIntake subAlgaeIntake;
  Climber subClimber;

  /** Creates a new none. */
  public None(CoralOuttake subCoralOuttake, Hopper subHopper, AlgaeIntake subAlgaeIntake, Climber subClimber) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.subCoralOuttake = subCoralOuttake;
    this.subHopper = subHopper;
    this.subAlgaeIntake = subAlgaeIntake
  ;
    this.subClimber = subClimber;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    subCoralOuttake.setCoralOuttake(0);
    subHopper.runHopper(0);
    subAlgaeIntake.setAlgaeIntakeMotor(0);
    subClimber.setClimberMotorVelocity(0);

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
