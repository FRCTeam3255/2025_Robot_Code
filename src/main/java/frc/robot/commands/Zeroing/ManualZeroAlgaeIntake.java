// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Zeroing;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.constAlgaeIntake;
import frc.robot.subsystems.AlgaeIntake;

public class ManualZeroAlgaeIntake extends Command {
  AlgaeIntake globalAlgaeIntake;

  boolean zeroingSuccess = false;
  Time zeroingTimestamp = Units.Seconds.of(0);

  AngularVelocity lastRotorVelocity = Units.RotationsPerSecond.of(0);

  public ManualZeroAlgaeIntake(AlgaeIntake subAlgaeIntake) {
    this.globalAlgaeIntake = subAlgaeIntake;

    addRequirements(subAlgaeIntake);
  }

  @Override
  public void initialize() {
    zeroingSuccess = false;
    globalAlgaeIntake.hasZeroed = false;
  }

  @Override
  public void execute() {
    // Check if we have raised the algae intake above a certain speed
    if (globalAlgaeIntake.getRotorVelocity().gte(constAlgaeIntake.MANUAL_ZEROING_START_VELOCITY)
        || globalAlgaeIntake.attemptingZeroing) {
      // Enter zeroing mode!
      if (!globalAlgaeIntake.attemptingZeroing) {
        globalAlgaeIntake.attemptingZeroing = true;
        zeroingTimestamp = Units.Seconds.of(Timer.getFPGATimestamp());
        System.out.println("Algae Intake Zeroing Started!");
      }

      // Check if time elapsed is too high (zeroing timeout)
      if (Units.Seconds.of(Timer.getFPGATimestamp()).minus(zeroingTimestamp).gte(constAlgaeIntake.ZEROING_TIMEOUT)) {
        globalAlgaeIntake.attemptingZeroing = false;
        System.out.println("Algae Intake Zeroing Failed :(");
      } else {
        boolean deltaRotorVelocity = globalAlgaeIntake.getRotorVelocity().minus(lastRotorVelocity)
            .lte(constAlgaeIntake.MANUAL_ZEROING_DELTA_VELOCITY);

        if (deltaRotorVelocity && lastRotorVelocity.lte(Units.RotationsPerSecond.of(0))) {
          zeroingSuccess = true;
        } else {
          lastRotorVelocity = globalAlgaeIntake.getRotorVelocity();
        }
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    globalAlgaeIntake.setSoftwareLimits(true, true);

    if (!interrupted && zeroingSuccess) {
      globalAlgaeIntake.hasZeroed = true;
      globalAlgaeIntake.resetSensorPosition(constAlgaeIntake.MAX_POS);
      System.out.println("Algae Intake Zeroing Successful!!!! Yippee and hooray!!! :3");
    } else {
      System.out.println("Algae Intake was never zeroed :((( blame eli");
    }
  }

  @Override
  public boolean isFinished() {
    return zeroingSuccess && globalAlgaeIntake.isRotorVelocityZero();
  }
}
