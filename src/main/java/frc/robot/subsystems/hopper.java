// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class hopper extends SubsystemBase {

  TalonFX hopperMotor;

  /** Creates a new hopper. */
  public hopper() {
    hopperMotor = new TalonFX(0);

  }

  public void runHopper(double speed) {
    hopperMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
