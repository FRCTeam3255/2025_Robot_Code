// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Zeroing;

import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.constElevator;
import frc.robot.subsystems.Elevator;

public class ManualZeroElevator extends Command {
  Elevator subElevator;

  boolean zeroingSuccess = false;
  Time zeroingTimestamp = Units.Seconds.of(0);

  AngularVelocity lastRotorVelocity = Units.RotationsPerSecond.of(0);

  public ManualZeroElevator(Elevator subElevator) {
    this.subElevator = subElevator;

    addRequirements(subElevator);
  }

  @Override
  public void initialize() {
    subElevator.setSoftwareLimits(false, true);
  }

  @Override
  public void execute() {
    // Check if we have raised the elevator above a certain speed
    if (subElevator.getRotorVelocity().gte(constElevator.MANUAL_ZEROING_START_VELOCITY)
        || Elevator.attemptingZeroing) {
      // Enter zeroing mode!
      if (!Elevator.attemptingZeroing) {
        Elevator.attemptingZeroing = true;
        zeroingTimestamp = Units.Seconds.of(Timer.getFPGATimestamp());
        System.out.println("Elevator Zeroing Started!");
      }

      // Check if time elapsed is too high (zeroing timeout)
      if (Units.Seconds.of(Timer.getFPGATimestamp()).minus(zeroingTimestamp).gte(constElevator.ZEROING_TIMEOUT)) {
        Elevator.attemptingZeroing = false;
        System.out.println("Elevator Zeroing Failed :(");
      } else {
        boolean deltaRotorVelocity = subElevator.getRotorVelocity().minus(lastRotorVelocity)
            .lte(constElevator.MANUAL_ZEROING_DELTA_VELOCITY);

        if (deltaRotorVelocity && lastRotorVelocity.lte(Units.RotationsPerSecond.of(0))) {
          zeroingSuccess = true;
        } else {
          lastRotorVelocity = subElevator.getRotorVelocity();
        }
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    subElevator.setSoftwareLimits(true, true);

    if (!interrupted) {
      Elevator.hasZeroed = true;
      subElevator.resetSensorPosition(constElevator.ZEROED_POS);
      System.out.println("Elevator Zeroing Successful!!!! Yippee and hooray!!! :3");
    } else {
      System.out.println("Elevator was never zeroed :((( blame eli");
    }
  }

  @Override
  public boolean isFinished() {
    boolean rotorVelocityIsZero = subElevator.getRotorVelocity().isNear(Units.RotationsPerSecond.zero(), 0.01);
    SmartDashboard.putBoolean("Zeroing/Pivot/Is Rotor Velocity Zero", rotorVelocityIsZero);
    return zeroingSuccess && rotorVelocityIsZero;
  }
}