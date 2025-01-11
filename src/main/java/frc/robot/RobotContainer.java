// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.frcteam3255.joystick.SN_XboxController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.constAlgaeIntake;
import frc.robot.Constants.constControllers;
import frc.robot.Constants.constCoralOuttake;
import frc.robot.RobotMap.mapControllers;
import frc.robot.commands.DriveManual;
import frc.robot.commands.ExampleAuto;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {

  private final SN_XboxController conDriver = new SN_XboxController(mapControllers.DRIVER_USB);
  private final SN_XboxController conOperator = new SN_XboxController(mapControllers.OPERATOR_USB);

  private final Drivetrain subDrivetrain = new Drivetrain();
  private final AlgaeIntake subAlgaeIntake = new AlgaeIntake();

  public RobotContainer() {
    conDriver.setLeftDeadband(constControllers.DRIVER_LEFT_STICK_DEADBAND);

    subDrivetrain
        .setDefaultCommand(
            new DriveManual(subDrivetrain, conDriver.axis_LeftY, conDriver.axis_LeftX, conDriver.axis_RightX));

    configureDriverBindings(conDriver);
    configureOperatorBindings(conOperator);

    subDrivetrain.resetModulesToAbsolute();
  }

  private void configureDriverBindings(SN_XboxController controller) {
    controller.btn_B.onTrue(Commands.runOnce(() -> subDrivetrain.resetModulesToAbsolute()));
    controller.btn_Back
        .onTrue(Commands.runOnce(() -> subDrivetrain.resetPoseToPose(new Pose2d(0, 0, new Rotation2d()))));

    // Defaults to Field-Relative, is Robot-Relative while held
    controller.btn_LeftBumper
        .whileTrue(Commands.runOnce(() -> subDrivetrain.setRobotRelative()))
        .onFalse(Commands.runOnce(() -> subDrivetrain.setFieldRelative()));
  }

  private void configureOperatorBindings(SN_XboxController controller) {
    // LT: Eat Algae
    controller.btn_LeftTrigger
        .onTrue(Commands.runOnce(() -> subAlgaeIntake.setAlgaeIntakeMotor(constAlgaeIntake.ALGAE_INTAKE_SPEED)))
        .onFalse(Commands.runOnce(() -> subAlgaeIntake.setAlgaeIntakeMotor(0)));
    // RT: Spit Algae
    controller.btn_RightTrigger
        .onTrue(Commands.runOnce(() -> subAlgaeIntake.setAlgaeIntakeMotor(constAlgaeIntake.ALGAE_OUTTAKE_SPEED)))
        .onFalse(Commands.runOnce(() -> subAlgaeIntake.setAlgaeIntakeMotor(0)));

    // RB: Score Coral
    controller.btn_RightBumper
        .onTrue(Commands.runOnce(() -> subAlgaeIntake.setAlgaeIntakeMotor(constCoralOuttake.CORAL_OUTTAKE_SPEED)))
        .onFalse(Commands.runOnce(() -> subAlgaeIntake.setAlgaeIntakeMotor(0)));
  }

  public Command getAutonomousCommand() {
    return Commands.runOnce(() -> subDrivetrain.resetPoseToPose(Constants.constField.WORKSHOP_STARTING_POSE))
        .andThen(new ExampleAuto(subDrivetrain));
  }
}
