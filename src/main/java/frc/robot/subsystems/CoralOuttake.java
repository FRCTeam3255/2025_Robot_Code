// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constCoralOuttake;
import frc.robot.RobotMap.mapCoralOuttake;

@Logged
public class CoralOuttake extends SubsystemBase {
  TalonFX outtakeMotor;
  TalonFX outtakeMotor2;
  CANrange coralSensor;
  boolean hasCoralOverride;

  /** Creates a new CoralOuttake. */
  public CoralOuttake() {
    outtakeMotor = new TalonFX(mapCoralOuttake.CORAL_OUTTAKE_LEFT_MOTOR_CAN);
    outtakeMotor2 = new TalonFX(mapCoralOuttake.CORAL_OUTTAKE_RIGHT_MOTOR_CAN);
    coralSensor = new CANrange(mapCoralOuttake.CORAL_SENSOR_CAN);

    hasCoralOverride = false;

    outtakeMotor.getConfigurator().apply(constCoralOuttake.CORAL_OUTTAKE_CONFIG);
    outtakeMotor2.getConfigurator().apply(constCoralOuttake.CORAL_OUTTAKE_CONFIG);
  }

  public void setCoralOuttake(double speed) {
    outtakeMotor.set(speed);
    outtakeMotor2.set(-speed);
  }

  public void setHasCoralOverride(boolean hasCoral) {
    this.hasCoralOverride = hasCoral;
  }

  public boolean hasCoral() {
    if (hasCoralOverride || coralSensor.getDistance().getValue().lt(constCoralOuttake.REQUIRED_CORAL_DISTANCE)) {
      return true;
    } else {
      return false;
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("CORAL SENSOR DISTANCE", coralSensor.getDistance().getValue().in(Units.Inches));
    SmartDashboard.putBoolean("CORAL SENSOR HAS GP", hasCoral());
    // This method will be called once per scheduler run
  }
}
