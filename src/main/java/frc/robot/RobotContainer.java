// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.frcteam3255.joystick.SN_XboxController;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.constAlgaeIntake;
import frc.robot.Constants.constControllers;
import frc.robot.Constants.constCoralOuttake;
import frc.robot.Constants.constField;
import frc.robot.RobotMap.mapControllers;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Climber;
import frc.robot.commands.states.Climb;
import frc.robot.commands.states.PlaceCoral;
import frc.robot.commands.states.PrepCoralLv;
import frc.robot.commands.states.PrepNet;
import frc.robot.commands.states.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class RobotContainer {

  private final SN_XboxController conDriver = new SN_XboxController(mapControllers.DRIVER_USB);
  private final SN_XboxController conOperator = new SN_XboxController(mapControllers.OPERATOR_USB);

  private final Drivetrain subDrivetrain = new Drivetrain();

  private final Hopper subHopper = new Hopper();

  private final AlgaeIntake subAlgaeIntake = new AlgaeIntake();
  private final CoralOuttake subCoralOuttake = new CoralOuttake();
  private final Climber subClimber = new Climber();
  private final Elevator subElevator = new Elevator();

  private final IntakeCoralHopper com_IntakeCoralHopper = new IntakeCoralHopper(subHopper, subCoralOuttake);
  private final Climb comClimb = new Climb(subClimber);
  private final PlaceCoral comPlaceCoral = new PlaceCoral(subCoralOuttake);
  private final PrepProcessor comPrepProcessor = new PrepProcessor(subElevator);
  private final PrepNet comPrepNet = new PrepNet(subElevator);
  private final CleaningL3Reef comCleaningL3Reef = new CleaningL3Reef(subElevator, subAlgaeIntake);
  private final CleaningL2Reef comCleaningL2Reef = new CleaningL2Reef(subElevator, subAlgaeIntake);

  public RobotContainer() {
    conDriver.setLeftDeadband(constControllers.DRIVER_LEFT_STICK_DEADBAND);

    subDrivetrain
        .setDefaultCommand(
            new DriveManual(subDrivetrain, conDriver.axis_LeftY, conDriver.axis_LeftX, conDriver.axis_RightX,
                conDriver.btn_LeftBumper));

    configureDriverBindings(conDriver);
    configureOperatorBindings(conOperator);

    subDrivetrain.resetModulesToAbsolute();

    NamedCommands.registerCommand("PrepPlace",
        Commands.sequence(
            Commands.runOnce(() -> subElevator.setPosition(Constants.constElevator.CORAL_L4_HEIGHT), subElevator)));

    NamedCommands.registerCommand("Place Sequence",
        Commands.sequence(
            Commands.runOnce(() -> subAlgaeIntake.setAlgaeIntakeMotor(constAlgaeIntake.ALGAE_OUTTAKE_SPEED)),
            Commands.waitSeconds(0.3),
            Commands.runOnce(() -> subElevator.setPosition(Constants.constElevator.CORAL_L1_HEIGHT), subElevator)));

    NamedCommands.registerCommand("Prep Coral Station",
        Commands.runOnce(() -> subElevator.setPosition(Constants.constElevator.CORAL_L4_HEIGHT), subElevator));

    NamedCommands.registerCommand("Get Coral Station Piece", new IntakeCoralHopper(subHopper, subCoralOuttake));
  }

  private void configureDriverBindings(SN_XboxController controller) {
    controller.btn_B.onTrue(Commands.runOnce(() -> subDrivetrain.resetModulesToAbsolute()));
    controller.btn_Back
        .onTrue(Commands.runOnce(() -> subDrivetrain.resetPoseToPose(constField.getFieldPositions().get()[0])));
  }

  private void configureOperatorBindings(SN_XboxController controller) {
    controller.btn_Start.onTrue(Commands.runOnce(() -> subElevator.resetSensorPosition(0)).ignoringDisable(true));
    controller.btn_Back.whileTrue(com_IntakeCoralHopper);
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

    controller.btn_East
        .onTrue(Commands.runOnce(() -> subElevator.setNeutral(), subElevator));
    // btn_South: Prep Processor
    controller.btn_South
        .onTrue(comPrepProcessor);

    // btn_West: Clean L3 Reef
    controller.btn_West
        .whileTrue(comCleaningL3Reef);

    // btn_North: Clean L2 Reef
    controller.btn_North
        .whileTrue(comCleaningL2Reef);

    // btn_A/B/Y/X: Set Elevator to Coral Levels
    controller.btn_A
        .onTrue(new PrepCoralLv(subElevator, Constants.constElevator.CORAL_L1_HEIGHT));
    controller.btn_B
        .onTrue(new PrepCoralLv(subElevator, Constants.constElevator.CORAL_L2_HEIGHT));
    controller.btn_Y
        .onTrue(new PrepCoralLv(subElevator, Constants.constElevator.CORAL_L3_HEIGHT));
    controller.btn_X
        .onTrue(new PrepCoralLv(subElevator, Constants.constElevator.CORAL_L4_HEIGHT));
  }

  public Command getAutonomousCommand() {
    // return new PathPlannerAuto("4-Piece-Low");
    return new PathPlannerAuto("L");
  }
}
