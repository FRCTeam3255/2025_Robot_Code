// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.constElevator;
import frc.robot.RobotMap.mapElevator;

@Logged
public class Elevator extends SubsystemBase {
  private TalonFX leftMotorFollower;
  private TalonFX rightMotorLeader;

  Distance currentLeftPosition = Units.Inches.of(0);
  Distance currentRightPosition = Units.Inches.of(0);
  private Distance lastDesiredPosition;

  /** Creates a new Elevator. */
  public Elevator() {
    leftMotorFollower = new TalonFX(mapElevator.ELEVATOR_LEFT_CAN);
    rightMotorLeader = new TalonFX(mapElevator.ELEVATOR_RIGHT_CAN);

    lastDesiredPosition = Units.Inches.of(0);

    rightMotorLeader.getConfigurator().apply(constElevator.ELEVATOR_CONFIG);
    leftMotorFollower.getConfigurator().apply(constElevator.ELEVATOR_CONFIG);
  }

  public Distance getElevatorPosition() {
    return Units.Inches.of(rightMotorLeader.getPosition().getValueAsDouble());
  }

  public Boolean isAtSetpoint() {
        return (getElevatorPosition().compareTo(getElevatorPosition().minus(Constants.constElevator.DEADZONE_DISTANCE)) > 0) &&
        getElevatorPosition().compareTo(getElevatorPosition().plus(Constants.constElevator.DEADZONE_DISTANCE)) < 0;
  }

  public Distance getLastDesiredPosition() {
    return lastDesiredPosition;
  }

  public void setPosition(Distance height) {
    rightMotorLeader.setControl(new PositionVoltage(height.in(Units.Inches)));
    leftMotorFollower.setControl(new Follower(rightMotorLeader.getDeviceID(), true));
    lastDesiredPosition = height;
  }

  public void setNeutral() {
    rightMotorLeader.setControl(new NeutralOut());
    leftMotorFollower.setControl(new NeutralOut());
  }

  public void resetSensorPosition(Distance setpoint) {
    rightMotorLeader.setPosition(setpoint.in(Inches));
    leftMotorFollower.setPosition(setpoint.in(Inches));

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    currentLeftPosition = Units.Inches.of(leftMotorFollower.getPosition().getValueAsDouble());
    currentRightPosition = Units.Inches.of(rightMotorLeader.getPosition().getValueAsDouble());
    
    SmartDashboard.putNumber("Elevator/Left/CLO", leftMotorFollower.getClosedLoopOutput().getValueAsDouble());
    SmartDashboard.putNumber("Elevator/Left/Output", leftMotorFollower.get());
    SmartDashboard.putNumber("Elevator/Left/Inverted", leftMotorFollower.getAppliedRotorPolarity().getValueAsDouble());
    SmartDashboard.putNumber("Elevator/Left/Current", leftMotorFollower.getSupplyCurrent().getValueAsDouble());

    SmartDashboard.putNumber("Elevator/Right/CLO", rightMotorLeader.getClosedLoopOutput().getValueAsDouble());
    SmartDashboard.putNumber("Elevator/Right/Output", rightMotorLeader.get());
    SmartDashboard.putNumber("Elevator/Right/Inverted", rightMotorLeader.getAppliedRotorPolarity().getValueAsDouble());
    SmartDashboard.putNumber("Elevator/Right/Current", rightMotorLeader.getSupplyCurrent().getValueAsDouble());
  }
}