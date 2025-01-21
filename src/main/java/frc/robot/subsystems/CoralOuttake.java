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
  boolean hasGamePieceCoral = false;

  /** Creates a new CoralOuttake. */
  public CoralOuttake() {
    outtakeMotor = new TalonFX(mapCoralOuttake.CORAL_OUTTAKE_LEFT_MOTOR_CAN);
    outtakeMotor2 = new TalonFX(mapCoralOuttake.CORAL_OUTTAKE_RIGHT_MOTOR_CAN);
    coralSensor = new CANrange(mapCoralOuttake.CORAL_SENSOR_CAN);

    outtakeMotor.getConfigurator().apply(constCoralOuttake.CORAL_OUTTAKE_CONFIG);
    outtakeMotor2.getConfigurator().apply(constCoralOuttake.CORAL_OUTTAKE_CONFIG);
  }

  public void setCoralOuttake(double speed) {
    outtakeMotor.set(speed);
    outtakeMotor2.set(-speed);
  }

  public void setHasGamePieceCoral(boolean hasCoral) {
    this.hasGamePieceCoral = hasCoral;
  }

  public boolean hasCoral() {
    if (coralSensor.getDistance().getValue().lt(constCoralOuttake.REQUIRED_CORAL_DISTANCE)) {
      hasGamePieceCoral = true;
    } else {
      hasGamePieceCoral = false;
    }
    return hasGamePieceCoral;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("CORAL OUTTAKE RPM",
        outtakeMotor.getVelocity().getValue().in(Units.RotationsPerSecond));
    SmartDashboard.putNumber("CORAL SENSOR DISTANCE", coralSensor.getDistance().getValue().in(Units.Inches));
    SmartDashboard.putBoolean("CORAL SENSOR HAS GP", hasCoral());

    // This method will be called once per scheduler run
  }
}
