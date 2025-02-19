// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.Constants.constClimber;

@Logged
public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  TalonFX climberMotor;
  Angle lastDesiredPosition;

  public Climber() {
    lastDesiredPosition = Units.Degrees.of(0);
    climberMotor = new TalonFX(RobotMap.mapClimber.CLIMBER_LEFT_CAN);

    climberMotor.getConfigurator().apply(constClimber.CLIMBER_CONFIG);
  }

  public void setClimberMotorVelocity(double velocity) {
    climberMotor.set(velocity);
  }

  public Distance getClimberPosition() {
    return Units.Inches.of(climberMotor.get());
  }

  public void setPosition(Angle angle) {
    climberMotor.setControl(new PositionVoltage(angle.in(Units.Rotations)));
    lastDesiredPosition = angle;
  }

  public void setNeutral() {
    climberMotor.setControl(new NeutralOut());
  }

  public void resetSensorPosition(Angle setpoint) {
    climberMotor.setPosition(setpoint.in(Rotations));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
