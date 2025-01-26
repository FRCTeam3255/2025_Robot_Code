// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Zeroing;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.constElevator;
import frc.robot.subsystems.Elevator;

public class ZeroElevator extends Command {
  Elevator subElevator;

  Time zeroingTimestamp;
  boolean hasZeroed = false;

  public ZeroElevator(Elevator subElevator) {
    this.subElevator = subElevator;

    addRequirements(subElevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    subElevator.setSoftwareLimits(false, true);

    subElevator.setVoltage(Units.Volts.zero());
    zeroingTimestamp = Units.Seconds.zero();
    hasZeroed = Elevator.hasZeroed;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    subElevator.setVoltage(constElevator.ZEROING_VOLTAGE);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subElevator.setSoftwareLimits(true, true);

    // Stop all movement
    subElevator.setVoltage(Units.Volts.zero());

    // Reset to the current position if this command was not interrupted
    if (!interrupted) {
      subElevator.resetSensorPosition(constElevator.ZEROED_POS);
      Elevator.hasZeroed = true;
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (hasZeroed) {
      return true;
    }

    // If the current velocity is low enough to be considered as zeroed
    if (subElevator.getRotorVelocity().lt(constElevator.ZEROED_VELOCITY)) {
      // And this is the first loop it has happened, begin the timer
      if (zeroingTimestamp.equals(Units.Seconds.zero())) {
        zeroingTimestamp = Units.Seconds.of(Timer.getFPGATimestamp());
        return false;
      }

      // If this isn't the first loop, return if it has been below the threshold for
      // long enough
      return (Units.Seconds.of(Timer.getFPGATimestamp()).minus(zeroingTimestamp).gte(constElevator.ZEROED_TIME));
    }

    // If the above wasn't true, we have gained too much velocity, so we aren't at 0
    // & need to restart the timer
    zeroingTimestamp = Units.Seconds.zero();
    return false;
  }
}