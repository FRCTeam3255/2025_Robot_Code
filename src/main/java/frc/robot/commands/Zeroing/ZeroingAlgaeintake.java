// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Zeroing;

import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.constAlgaeIntake;
import frc.robot.subsystems.AlgaeIntake;

public class ZeroingAlgaeintake extends Command {
  AlgaeIntake subAlgaeIntake;

  boolean zeroingSuccess = false;
  Measure<TimeUnit> zeroingTimestamp = Units.Seconds.of(0);

  Measure<AngularVelocityUnit> lastRotorVelocity = Units.RotationsPerSecond.of(0);

  public void ManualZeroAlgaeintake(AlgaeIntake subAlgaeIntake) {
    this.subAlgaeIntake = subAlgaeIntake;

    addRequirements(subAlgaeIntake);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    // Check if we have raised the AlgaeIntake above a certain speed
    if (subAlgaeIntake.getRotorVelocity().gte(constAlgaeIntake.MANUAL_ZEROING_START_VELOCITY)
        || AlgaeIntake.attemptingZeroing) {
      // Enter zeroing mode!
      if (!AlgaeIntake.attemptingZeroing) {
        AlgaeIntake.attemptingZeroing = true;
        zeroingTimestamp = Units.Seconds.of(Timer.getFPGATimestamp());
        System.out.println("AlgaeIntake Zeroing Started!");
      }

      // Check if time elapsed is too high (zeroing timeout)
      if (Units.Seconds.of(Timer.getFPGATimestamp()).minus(zeroingTimestamp).gte(constAlgaeIntake.ZEROING_TIMEOUT)) {
        AlgaeIntake.attemptingZeroing = false;
        System.out.println("AlgaeIntake Zeroing Failed :(");
      } else {
        boolean deltaRotorVelocity = subAlgaeIntake.getRotorVelocity().minus(lastRotorVelocity)
            .lte(constAlgaeIntake.MANUAL_ZEROING_DELTA_VELOCITY);

        if (deltaRotorVelocity && lastRotorVelocity.lte(Units.RotationsPerSecond.of(0))) {
          zeroingSuccess = true;
        } else {
          lastRotorVelocity = subAlgaeIntake.getRotorVelocity();
        }
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (!interrupted) {
      AlgaeIntake.hasZeroed = true;
      subAlgaeIntake.setSensorPosition(constAlgaeIntake.ZEROED_POS);
      System.out.println("AlgaeIntake Zeroing Successful!!!! Yippee and hooray!!! :3");
    } else {
      System.out.println("AlgaeIntake was never zeroed :((( Blame Eli");
    }
  }

  @Override
  public boolean isFinished() {
    boolean rotorVelocityIsZero = subAlgaeIntake.getRotorVelocity().isNear(Units.RotationsPerSecond.zero(), 0.01);
    SmartDashboard.putBoolean("Zeroing/Pivot/Is Rotor Velocity Zero", rotorVelocityIsZero);
    return zeroingSuccess && rotorVelocityIsZero;
  }
}