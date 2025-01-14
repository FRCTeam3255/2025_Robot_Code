// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.mapAlgaeIntake;
import frc.robot.Constants.constAlgaeIntake;

public class AlgaeIntake extends SubsystemBase {
  TalonFX intakeMotor;
  TalonFXConfiguration intakeConfig;
  double intakeHasGamePieceVelocity = constAlgaeIntake.ALGAE_INTAKE_VEOLOCITY;
  double intakeHasGamePieceCurrent = constAlgaeIntake.ALGAE_INTAKE_CURRENT;

  /** Creates a new AlgaeIntake. */
  public AlgaeIntake() {
    intakeMotor = new TalonFX(mapAlgaeIntake.ALGAE_MOTOR_CAN);

    intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = constAlgaeIntake.ALGAE_INTAKE_CURRENT_LIMIT_ENABLE;
    intakeConfig.CurrentLimits.SupplyCurrentLimit = constAlgaeIntake.ALGAE_INTAKE_CURRENT_LIMIT;
    intakeConfig.CurrentLimits.SupplyCurrentLowerLimit = constAlgaeIntake.ALGAE_INTAKE_CURRENT_THRESHOLD;
    intakeConfig.CurrentLimits.SupplyCurrentLowerTime = constAlgaeIntake.ALGAE_INTAKE_TIME_THRESHOLD;

    intakeMotor.getConfigurator().apply(intakeConfig);

  }

  public boolean hasGamePiece = true;

  public void setAlgaeIntakeMotor(double speed) {
    intakeMotor.set(speed);
  }

  public void HasAlgae(boolean enabled) {
    double intakeCurrent = intakeMotor.getStatorCurrent().getValueAsDouble();

    double intakeVelocity = intakeMotor.getVelocity().getValueAsDouble();

    intakeHasGamePieceCurrent = constAlgaeIntake.ALGAE_INTAKE_CURRENT;
    intakeHasGamePieceVelocity = constAlgaeIntake.ALGAE_INTAKE_VEOLOCITY;

    if (hasGamePiece || (intakeCurrent >= intakeHasGamePieceCurrent)
        && (intakeVelocity <= intakeHasGamePieceVelocity)) {
      hasGamePiece = true;
    } else {
      hasGamePiece = false;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
