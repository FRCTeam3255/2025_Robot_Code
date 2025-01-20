// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.mapAlgaeIntake;
import frc.robot.Constants.constAlgaeIntake;

@Logged
public class AlgaeIntake extends SubsystemBase {
  TalonFX intakeMotorOne;
  TalonFX intakeMotorTwo;
  TalonFXConfiguration intakeConfig;
  AngularVelocity intakeHasGamePieceVelocity = constAlgaeIntake.ALGAE_INTAKE_HAS_GP_VELOCITY;;
  Current intakeHasGamePieceCurrent = constAlgaeIntake.ALGAE_INTAKE_HAS_GP_CURRENT;

  /** Creates a new AlgaeIntake. */
  public AlgaeIntake() {
    intakeMotorOne = new TalonFX(mapAlgaeIntake.INTAKE_LEFT_MOTOR_CAN);
    intakeMotorTwo = new TalonFX(mapAlgaeIntake.INTAKE_RIGHT_MOTOR_CAN);

    intakeMotorOne.getConfigurator().apply(constAlgaeIntake.ALGAE_INTAKE_CONFIG);
    intakeMotorTwo.getConfigurator().apply(constAlgaeIntake.ALGAE_INTAKE_CONFIG);
  }

  public boolean hasGamePiece = false;

  public void setAlgaeIntakeMotor(double speed) {
    intakeMotorOne.set(speed);
    intakeMotorTwo.set(speed);
  }

  public boolean hasAlgae() {
    Current intakeCurrent = intakeMotorOne.getStatorCurrent().getValue();

    AngularVelocity intakeVelocity = intakeMotorOne.getVelocity().getValue();
    double intakeAcceleration = intakeMotorOne.getAcceleration().getValueAsDouble();

    intakeHasGamePieceCurrent = constAlgaeIntake.ALGAE_INTAKE_HAS_GP_CURRENT;
    intakeHasGamePieceVelocity = constAlgaeIntake.ALGAE_INTAKE_HAS_GP_VELOCITY;

    if (hasGamePiece || (intakeCurrent.gte(intakeHasGamePieceCurrent))
        && (intakeVelocity.gte(intakeHasGamePieceVelocity)) && (intakeVelocity.lt(Units.RotationsPerSecond.zero()))
        && (intakeAcceleration < 0)) {
      hasGamePiece = true;
    } else {
      hasGamePiece = false;
    }
    return hasGamePiece;
  }

  public void setHasGamePiece(boolean passedHasGamePiece) {
    hasGamePiece = passedHasGamePiece;
  }

  public double getAlgaeIntakeVoltage() {
    return intakeMotorOne.getMotorVoltage().getValueAsDouble();
  }

  public void setAlgaeIntakeVoltage(double voltage) {
    intakeMotorOne.setVoltage(voltage);
    intakeMotorTwo.setVoltage(voltage);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("RightAlgae motor", intakeMotorOne.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("LeftAlgae motor", intakeMotorTwo.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("LeftAlgae motor velocity", intakeMotorOne.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("LeftAlgae motor Voltage", intakeMotorOne.getMotorVoltage().getValueAsDouble());

    SmartDashboard.putBoolean("Has Algae", hasAlgae());
    // This method will be called once per scheduler run
  }
}
