// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.constElevator;
import frc.robot.subsystems.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CoralStuckSoftwareLimitToggle extends InstantCommand {
  Elevator subElevator;
  boolean coralStuckToggle = false;

  public CoralStuckSoftwareLimitToggle(Elevator subElevator) {
    this.subElevator = subElevator;

    addRequirements(subElevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    coralStuckToggle = !coralStuckToggle;
    if (coralStuckToggle) {
      subElevator.setSoftwareLimits(constElevator.CORAL_STUCK_REVERSE_LIMIT.in(Units.Inches),
          constElevator.NORMAL_FORWARD_LIMIT.in(Units.Inches));
    } else {
      subElevator.setSoftwareLimits(constElevator.NORMAL_REVERSE_LIMIT.in(Units.Inches),
          constElevator.NORMAL_FORWARD_LIMIT.in(Units.Inches));
    }
  }
}
