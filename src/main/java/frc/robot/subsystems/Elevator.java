// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.constElevator;
import frc.robot.RobotMap.mapElevator;

@Logged
public class Elevator extends SubsystemBase {
  private TalonFX leftMotorFollower;
  private TalonFX rightMotorLeader;

  private Distance lastDesiredPosition;

  Distance currentLeftPosition = Units.Inches.of(0);
  Distance currentRightPosition = Units.Inches.of(0);

  PositionVoltage positionRequest;
  VoltageOut voltageRequest = new VoltageOut(0);

  public boolean attemptingZeroing = false;
  public boolean hasZeroed = false;

  MotionMagicVoltage motionRequest;

  /** Creates a new Elevator. */
  public Elevator() {
    leftMotorFollower = new TalonFX(mapElevator.ELEVATOR_LEFT_CAN);
    rightMotorLeader = new TalonFX(mapElevator.ELEVATOR_RIGHT_CAN);

    lastDesiredPosition = Units.Inches.of(0);
    voltageRequest = new VoltageOut(0);
    motionRequest = new MotionMagicVoltage(0);

    rightMotorLeader.getConfigurator().apply(constElevator.ELEVATOR_CONFIG);
    leftMotorFollower.getConfigurator().apply(constElevator.ELEVATOR_CONFIG);
  }

  public Distance getElevatorPosition() {
    return Units.Inches.of(rightMotorLeader.getPosition().getValueAsDouble());
  }

  public boolean isAtSetpoint() {
    return (getElevatorPosition()
        .compareTo(getLastDesiredPosition().minus(Constants.constElevator.DEADZONE_DISTANCE)) > 0) &&
        getElevatorPosition().compareTo(getLastDesiredPosition().plus(Constants.constElevator.DEADZONE_DISTANCE)) < 0;
  }

  public boolean isAtSpecificSetpoint(Distance setpoint) {
    return (getElevatorPosition()
        .compareTo(setpoint.minus(Constants.constElevator.DEADZONE_DISTANCE)) > 0) &&
        getElevatorPosition().compareTo(setpoint.plus(Constants.constElevator.DEADZONE_DISTANCE)) < 0;
  }

  public boolean isAtAnyScoringPosition() {
    if (isAtSpecificSetpoint(constElevator.CORAL_L1_HEIGHT) ||
        isAtSpecificSetpoint(constElevator.CORAL_L2_HEIGHT) ||
        isAtSpecificSetpoint(constElevator.CORAL_L3_HEIGHT) ||
        isAtSpecificSetpoint(constElevator.CORAL_L4_HEIGHT) ||
        isAtSpecificSetpoint(constElevator.ALGAE_PREP_NET) ||
        isAtSpecificSetpoint(constElevator.ALGAE_PREP_PROCESSOR_HEIGHT)) {
      return true;
    }
    return false;
  }

  public AngularVelocity getRotorVelocity() {
    return rightMotorLeader.getRotorVelocity().getValue();
  }

  public Distance getLastDesiredPosition() {
    return lastDesiredPosition;
  }

  public void setCoastMode(Boolean coastMode) {
    if (coastMode) {
      rightMotorLeader.getConfigurator().apply(constElevator.COAST_MODE_CONFIGURATION);
      leftMotorFollower.getConfigurator().apply(constElevator.COAST_MODE_CONFIGURATION);
    } else {
      rightMotorLeader.getConfigurator().apply(constElevator.ELEVATOR_CONFIG);
      leftMotorFollower.getConfigurator().apply(constElevator.ELEVATOR_CONFIG);
    }
  }

  public boolean isRotorVelocityZero() {
    return getRotorVelocity().isNear(Units.RotationsPerSecond.zero(), 0.01);
  }

  public void setPosition(Distance height) {
    rightMotorLeader.setControl(motionRequest.withPosition(height.in(Units.Inches)));
    leftMotorFollower.setControl(new Follower(rightMotorLeader.getDeviceID(), true));
    lastDesiredPosition = height;
  }

  public void setNeutral() {
    rightMotorLeader.setControl(new NeutralOut());
    leftMotorFollower.setControl(new NeutralOut());
  }

  public void setVoltage(Voltage voltage) {
    rightMotorLeader.setControl(voltageRequest.withOutput(voltage));
    leftMotorFollower.setControl(new Follower(rightMotorLeader.getDeviceID(), true));
  }

  public void setSoftwareLimits(boolean reverseLimitEnable, boolean forwardLimitEnable) {
    constElevator.ELEVATOR_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitEnable = reverseLimitEnable;
    constElevator.ELEVATOR_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitEnable = forwardLimitEnable;

    rightMotorLeader.getConfigurator().apply(constElevator.ELEVATOR_CONFIG);
    leftMotorFollower.getConfigurator().apply(constElevator.ELEVATOR_CONFIG);
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