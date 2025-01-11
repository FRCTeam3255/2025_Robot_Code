// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.mapCoralOuttake;

public class CoralOuttake extends SubsystemBase {
  TalonFX outtakeMotor;
  TalonFX outtakeMotor2;

  /** Creates a new CoralOuttake. */
  public CoralOuttake() {
    outtakeMotor = new TalonFX(mapCoralOuttake.CORAL_OUTTAKE_MOTOR_CAN);
    outtakeMotor2 = new TalonFX(mapCoralOuttake.CORAL_OUTTAKE_MOTOR_CAN_2);
  }

  public void setCoralOuttake(double speed) {
    outtakeMotor.set(speed);
  }

  public void setCoralOuttake2(double speed) {
    outtakeMotor2.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
