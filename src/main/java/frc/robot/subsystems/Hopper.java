// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constHopper;
import frc.robot.RobotMap.mapHopper;

@Logged
public class Hopper extends SubsystemBase {

  TalonFX hopperRollerMotor;
  TalonFX hopperPivotMotor;
  DigitalInput hopperSensor;

  private Angle lastDesiredAngle = Degrees.zero();

  PositionVoltage positionRequest;
  VoltageOut voltageRequest = new VoltageOut(0);
  MotionMagicVoltage motionRequest = new MotionMagicVoltage(0);

  /** Creates a new hopper. */
  public Hopper() {
    hopperRollerMotor = new TalonFX(mapHopper.HOPPER_MOTOR_CAN);
    hopperSensor = new DigitalInput(mapHopper.HOPPER_SENSOR_DIO);
    hopperPivotMotor = new TalonFX(mapHopper.HOPPER_PIVOT_MOTOR_CAN);

    hopperRollerMotor.getConfigurator().apply(constHopper.HOPPER_ROLLER_CONFIG);
    hopperPivotMotor.getConfigurator().apply(constHopper.HOPPER_PIVOT_CONFIG);
  }

  public void runHopper(double speed) {
    hopperRollerMotor.set(speed);
  }

  public void setHopperPivot(Angle setPoint) {
    hopperPivotMotor.setControl(motionRequest.withPosition(setPoint));
    lastDesiredAngle = setPoint;
  }

  public boolean getHopperSensor() {
    return hopperSensor.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
