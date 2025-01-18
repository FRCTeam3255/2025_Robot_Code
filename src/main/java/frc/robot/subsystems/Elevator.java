// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.constElevator;
import frc.robot.RobotMap.mapElevator;

@Logged
public class Elevator extends SubsystemBase {
  private TalonFX leftMotorFollower;
  private TalonFX rightMotorLeader;

  /** Creates a new Elevator. */
  public Elevator() {
    leftMotorFollower = new TalonFX(mapElevator.LEFT_ELEVATOR_CAN);
    rightMotorLeader = new TalonFX(mapElevator.RIGHT_ELEVATOR_CAN);

    configure();
  }

  public void configure() {
    rightMotorLeader.getConfigurator().apply(constElevator.ELEVATOR_CONFIG);
    leftMotorFollower.getConfigurator().apply(constElevator.ELEVATOR_CONFIG);
  }

  public Distance getElevatorPosition() {
    return Units.Inches.of(rightMotorLeader.get());
  }

  public void setPosition(Distance height) {
    rightMotorLeader.setControl(new PositionVoltage(height.in(Units.Inches)));
    leftMotorFollower.setControl(new Follower(rightMotorLeader.getDeviceID(), true));
  }

  public void setNeutral() {
    rightMotorLeader.setControl(new NeutralOut());
    leftMotorFollower.setControl(new NeutralOut());
  }

  public void setVoltage(Voltage volts) {
    rightMotorLeader.setControl(new VoltageOut(volts.in(Units.Volts)));
    leftMotorFollower.setControl(new VoltageOut(volts.in(Units.Volts)));
  }

  public void resetSensorPosition(double setpoint) {
    rightMotorLeader.setPosition(setpoint);
    leftMotorFollower.setPosition(setpoint);

  }

  final SysIdRoutine sysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
          null, // Use default ramp rate (1 V/s)
          Units.Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
          null, // Use default timeout (10 s)
                // Log state with Phoenix SignalLogger class
          (state) -> SignalLogger.writeString("state", state.toString())),
      new SysIdRoutine.Mechanism(
          (volts) -> setVoltage(volts),
          null,
          this));

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Elevator/Left/Pos", leftMotorFollower.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Elevator/Left/CLO", leftMotorFollower.getClosedLoopOutput().getValueAsDouble());
    SmartDashboard.putNumber("Elevator/Left/Output", leftMotorFollower.get());
    SmartDashboard.putNumber("Elevator/Left/Inverted", leftMotorFollower.getAppliedRotorPolarity().getValueAsDouble());
    SmartDashboard.putNumber("Elevator/Left/Current", leftMotorFollower.getSupplyCurrent().getValueAsDouble());

    SmartDashboard.putNumber("Elevator/Right/Pos", rightMotorLeader.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Elevator/Right/CLO", rightMotorLeader.getClosedLoopOutput().getValueAsDouble());
    SmartDashboard.putNumber("Elevator/Right/Output", rightMotorLeader.get());
    SmartDashboard.putNumber("Elevator/Right/Inverted", rightMotorLeader.getAppliedRotorPolarity().getValueAsDouble());
    SmartDashboard.putNumber("Elevator/Right/Current", rightMotorLeader.getSupplyCurrent().getValueAsDouble());

  }
}