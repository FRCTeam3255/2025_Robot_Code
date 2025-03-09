// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constHopper;
import frc.robot.RobotMap;
import frc.robot.RobotMap.mapHopper;

@Logged
public class Hopper extends SubsystemBase {

  TalonFX hopperMotor;
  DigitalInput hopperSensor;

  /** Creates a new hopper. */
  public Hopper() {
    hopperMotor = new TalonFX(mapHopper.HOPPER_MOTOR_CAN, RobotMap.CAN_BUS_MECHANISMS);
    hopperSensor = new DigitalInput(mapHopper.HOPPER_SENSOR_DIO);

    hopperMotor.getConfigurator().apply(constHopper.HOPPER_CONFIG);

  }

  public void runHopper(double speed) {
    hopperMotor.set(speed);
  }

  public boolean getHopperSensor() {
    return hopperSensor.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
