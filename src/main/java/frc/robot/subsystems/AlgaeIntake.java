// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.mapAlgaeIntake;
import frc.robot.Constants.constAlgaeIntake;

public class AlgaeIntake extends SubsystemBase {
  TalonFX intakeMotor;
  CANrange algaeSensor;

  /** Creates a new AlgaeIntake. */
  public AlgaeIntake() {
    intakeMotor = new TalonFX(mapAlgaeIntake.ALGAE_MOTOR_CAN);

    intakeMotor.getConfigurator().apply(constAlgaeIntake.ALGAE_INTAKE_CONFIG);

    algaeSensor = new CANrange(mapAlgaeIntake.ALGAE_SENSOR_CAN);
  }

  public void setAlgaeIntakeMotor(double speed) {
    intakeMotor.set(speed);
  }

  public boolean hasAlgae() {
    return algaeSensor.getDistance().getValue().lt(constAlgaeIntake.REQUIRED_ALGAE_DISTANCE);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
