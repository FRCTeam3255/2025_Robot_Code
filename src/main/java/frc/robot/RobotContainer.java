// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.frcteam3255.joystick.SN_XboxController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.constAlgaeIntake;
import frc.robot.Constants.constControllers;
import frc.robot.Constants.constCoralOuttake;
import frc.robot.Constants.constField;
import frc.robot.RobotMap.mapControllers;
import frc.robot.commands.Climb;
import frc.robot.commands.DriveManual;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Climber;
import frc.robot.commands.PlaceCoral;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class RobotContainer {

  private final SN_XboxController conDriver = new SN_XboxController(mapControllers.DRIVER_USB);
  private final SN_XboxController conOperator = new SN_XboxController(mapControllers.OPERATOR_USB);

  private final Drivetrain subDrivetrain = new Drivetrain();

  private final Hopper subHopper = new Hopper();
  private final IntakeHopper com_IntakeHopper = new IntakeHopper(subHopper);

  private final AlgaeIntake subAlgaeIntake = new AlgaeIntake();
  private final CoralOuttake subCoralOuttake = new CoralOuttake();
  private final Climber subClimber = new Climber();

  private final Climb comClimb = new Climb(subClimber);

  private final PlaceCoral comPlaceCoral = new PlaceCoral(subCoralOuttake);
  private final Elevator subElevator = new Elevator();

  public RobotContainer() {
    conDriver.setLeftDeadband(constControllers.DRIVER_LEFT_STICK_DEADBAND);

    subDrivetrain
        .setDefaultCommand(
            new DriveManual(subDrivetrain, conDriver.axis_LeftY, conDriver.axis_LeftX, conDriver.axis_RightX,
                conDriver.btn_LeftBumper));

    configureDriverBindings(conDriver);
    configureOperatorBindings(conOperator);

    subDrivetrain.resetModulesToAbsolute();
  }

  private void configureDriverBindings(SN_XboxController controller) {
    controller.btn_B.onTrue(Commands.runOnce(() -> subDrivetrain.resetModulesToAbsolute()));
    controller.btn_Back
        .onTrue(Commands.runOnce(() -> subDrivetrain.resetPoseToPose(constField.getFieldPositions().get()[0])));
  }

  private void configureOperatorBindings(SN_XboxController controller) {
    controller.btn_Back.whileTrue(com_IntakeHopper);
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
        .whileTrue(comPlaceCoral);

    // LB: Climb
    controller.btn_LeftBumper
        .whileTrue(comClimb);
    // btn_A/B/Y/X: Set Elevator to Coral Levels
    controller.btn_A
        .onTrue(Commands.runOnce(() -> subElevator.setPosition(Constants.constElevator.CORAL_L1_HEIGHT), subElevator));
    controller.btn_B
        .onTrue(Commands.runOnce(() -> subElevator.setPosition(Constants.constElevator.CORAL_L2_HEIGHT), subElevator));
    controller.btn_Y
        .onTrue(Commands.runOnce(() -> subElevator.setPosition(Constants.constElevator.CORAL_L3_HEIGHT), subElevator));
    controller.btn_X
        .onTrue(Commands.runOnce(() -> subElevator.setPosition(Constants.constElevator.CORAL_L4_HEIGHT), subElevator));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No auto selected :<");
  }
}
