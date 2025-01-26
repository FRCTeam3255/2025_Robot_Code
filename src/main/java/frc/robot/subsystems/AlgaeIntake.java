// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.mapAlgaeIntake;
import frc.robot.Constants.constAlgaeIntake;

@Logged
public class AlgaeIntake extends SubsystemBase {
  TalonFX intakeRollerMotor;
  TalonFX intakePivotMotor;

  AngularVelocity intakeHasGamePieceVelocity = constAlgaeIntake.ALGAE_INTAKE_HAS_GP_VELOCITY;
  Current intakeHasGamePieceCurrent = constAlgaeIntake.ALGAE_INTAKE_HAS_GP_CURRENT;
  private Angle lastDesiredAngle = Degrees.zero();

  MotionMagicVoltage motionRequest;

  /** Creates a new AlgaeIntake. */
  public AlgaeIntake() {
    intakeRollerMotor = new TalonFX(mapAlgaeIntake.INTAKE_ROLLER_MOTOR_CAN);
    intakePivotMotor = new TalonFX(mapAlgaeIntake.INTAKE_PIVOT_MOTOR_CAN);

    intakeRollerMotor.getConfigurator().apply(constAlgaeIntake.ALGAE_INTAKE_CONFIG);
    intakePivotMotor.getConfigurator().apply(constAlgaeIntake.ALGAE_PIVOT_CONFIG);
  }

  public boolean hasGamePiece = false;

  public void setAlgaeIntakeMotor(double speed) {
    intakeRollerMotor.set(speed);
  }

  public void setAlgaePivotAngle(Angle setpoint) {
    intakePivotMotor.setControl(motionRequest.withPosition(setpoint));
    lastDesiredAngle = setpoint;
  }

  public Angle getPivotAngle() {
    return intakePivotMotor.getPosition().getValue();
  }

  public Angle getLastDesiredPivotAngle() {
    return lastDesiredAngle;
  }

  public boolean hasAlgae() {
    Current intakeCurrent = intakeRollerMotor.getStatorCurrent().getValue();

    AngularVelocity intakeVelocity = intakeRollerMotor.getVelocity().getValue();
    double intakeAcceleration = intakeRollerMotor.getAcceleration().getValueAsDouble();

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
    return intakeRollerMotor.getMotorVoltage().getValueAsDouble();
  }

  public void setAlgaeIntakeVoltage(double voltage) {
    intakeRollerMotor.setVoltage(voltage);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Algae Intake/Roller/Stator Current",
        intakeRollerMotor.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Algae Intake/Roller/Velocity", intakeRollerMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Algae Intake/Roller/Voltage", intakeRollerMotor.getMotorVoltage().getValueAsDouble());

    SmartDashboard.putNumber("Algae Intake/Pivot/Stator Current",
        intakePivotMotor.getStatorCurrent().getValueAsDouble());

    SmartDashboard.putBoolean("Has Algae", hasAlgae());
  }
}
