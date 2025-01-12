// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constCoralOuttake;
import frc.robot.RobotMap.mapCoralOuttake;

public class CoralOuttake extends SubsystemBase {
  TalonFX outtakeMotor;
  TalonFX outtakeMotor2;
  CANrange coralSensor;

  /** Creates a new CoralOuttake. */
  public CoralOuttake() {
    outtakeMotor = new TalonFX(mapCoralOuttake.CORAL_OUTTAKE_MOTOR_CAN);
    outtakeMotor2 = new TalonFX(mapCoralOuttake.CORAL_OUTTAKE_MOTOR_CAN_2);
    coralSensor = new CANrange(mapCoralOuttake.CORAL_SENSOR_CAN);
  }

  public void setCoralOuttake(double speed) {
    outtakeMotor.set(speed);
    outtakeMotor2.set(-speed);
  }

  public boolean hasCoral() {
    return coralSensor.getDistance().getValue().lt(constCoralOuttake.REQUIRED_CORAL_DISTANCE);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
