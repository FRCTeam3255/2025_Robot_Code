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
  private boolean hasCoral, indexingCoral;

  /** Creates a new CoralOuttake. */
  public CoralOuttake() {
    outtakeMotor = new TalonFX(mapCoralOuttake.CORAL_OUTTAKE_LEFT_MOTOR_CAN);
    outtakeMotor2 = new TalonFX(mapCoralOuttake.CORAL_OUTTAKE_RIGHT_MOTOR_CAN);
    coralSensor = new CANrange(mapCoralOuttake.CORAL_SENSOR_CAN);

    hasCoral = false;

    outtakeMotor.getConfigurator().apply(constCoralOuttake.CORAL_OUTTAKE_CONFIG);
    outtakeMotor2.getConfigurator().apply(constCoralOuttake.CORAL_OUTTAKE_CONFIG);
    coralSensor.getConfigurator().apply(constCoralOuttake.CORAL_SENSOR_CONFIG);
  }

  public void setCoralOuttake(double speed) {
    outtakeMotor.set(speed);
    outtakeMotor2.set(-speed);
  }

  public void setIndexingCoral(boolean indexing) {
    this.indexingCoral = indexing;
  }

  public boolean isIndexingCoral() {
    return indexingCoral;
  }

  public void setHasCoral(boolean hasCoral) {
    this.hasCoral = hasCoral;
  }

  public void coralToggle() {
    this.hasCoral = !hasCoral;
  }

  public boolean sensorSeesCoral() {
    return coralSensor.getIsDetected().getValue();
  }

  public boolean sensorIndexedCoral() {
    return coralSensor.getDistance().getValue().gte(constCoralOuttake.INDEXED_CORAL_DISTANCE);
  }

  public boolean hasCoral() {
    return hasCoral;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("CORAL SENSOR DISTANCE", coralSensor.getDistance().getValue().in(Units.Inches));
    SmartDashboard.putBoolean("CORAL SENSOR HAS GP", hasCoral());
    // This method will be called once per scheduler run
  }
}
