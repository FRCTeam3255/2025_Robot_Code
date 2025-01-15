// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.frcteam3255.joystick.SN_XboxController;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.constAlgaeIntake;
import frc.robot.Constants.constControllers;
import frc.robot.Constants.constField;
import frc.robot.RobotMap.mapControllers;
import frc.robot.commands.states.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.StateMachine.RobotState;

public class RobotContainer {

  private final SN_XboxController conDriver = new SN_XboxController(mapControllers.DRIVER_USB);
  private final SN_XboxController conOperator = new SN_XboxController(mapControllers.OPERATOR_USB);

  private static final Drivetrain subDrivetrain = new Drivetrain();
  private static final Hopper subHopper = new Hopper();
  private static final Vision subVision = new Vision();

  private final AlgaeIntake subAlgaeIntake = new AlgaeIntake();
  private final CoralOuttake subCoralOuttake = new CoralOuttake();
  private final Climber subClimber = new Climber();
  private final Elevator subElevator = new Elevator();
  private final StateMachine subStateMachine = new StateMachine(subAlgaeIntake, subClimber, subCoralOuttake,
      subDrivetrain, subElevator, subHopper);

  private final IntakeCoralHopper com_IntakeCoralHopper = new IntakeCoralHopper(subStateMachine, subHopper,
      subCoralOuttake);
  private final Climb comClimb = new Climb(subStateMachine, subClimber);
  private final PlaceCoral comPlaceCoral = new PlaceCoral(subStateMachine, subCoralOuttake);
  private final PrepProcessor comPrepProcessor = new PrepProcessor(subStateMachine, subElevator);
  private final PrepNet comPrepNet = new PrepNet(subStateMachine, subElevator);
  private final CleaningL3Reef comCleaningL3Reef = new CleaningL3Reef(subStateMachine, subElevator, subAlgaeIntake);
  private final CleaningL2Reef comCleaningL2Reef = new CleaningL2Reef(subStateMachine, subElevator, subAlgaeIntake);
  private final IntakingAlgaeGround comIntakingAlgaeGround = new IntakingAlgaeGround(subStateMachine, subElevator,
      subAlgaeIntake);
  private final EjectingAlgae comEjectingAlgae = new EjectingAlgae(subStateMachine, subAlgaeIntake);

  private final Trigger hasCoralTrigger = new Trigger(subCoralOuttake::hasCoral);
  private final Trigger hasAlgaeTrigger = new Trigger(subAlgaeIntake::hasAlgae);

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

    NamedCommands.registerCommand("Get Coral Station Piece",
        new IntakeCoralHopper(subStateMachine, subHopper, subCoralOuttake));
  }

  private void configureDriverBindings(SN_XboxController controller) {
    controller.btn_B.onTrue(Commands.runOnce(() -> subDrivetrain.resetModulesToAbsolute()));
    controller.btn_Back
        .onTrue(Commands.runOnce(() -> subDrivetrain.resetPoseToPose(constField.getFieldPositions().get()[0])));
  }

  private void configureOperatorBindings(SN_XboxController controller) {

    // Start: Reset Elevator Sensor Position
    controller.btn_Start.onTrue(Commands.runOnce(() -> subElevator.resetSensorPosition(0))
        .ignoringDisable(true));

    // Back: Intake Coral
    controller.btn_Back
        .whileTrue(Commands.deferredProxy(
            () -> subStateMachine.tryState(RobotState.INTAKING_CORAL_HOPPER)))
        .onFalse(Commands.deferredProxy(
            () -> subStateMachine.tryState(RobotState.NONE)));

    // LT: Eat Algae
    controller.btn_LeftTrigger
        .whileTrue(Commands.deferredProxy(
            () -> subStateMachine.tryState(RobotState.INTAKING_ALGAE_GROUND)))
        .onFalse(Commands.deferredProxy(
            () -> subStateMachine.tryState(RobotState.NONE)));

    // RT: Spit Algae
    controller.btn_RightTrigger
        .onTrue(Commands.deferredProxy(
            () -> subStateMachine.tryState(RobotState.EJECTING_ALGAE)))
        .onFalse(Commands.deferredProxy(
            () -> subStateMachine.tryState(RobotState.NONE)));

    // RB: Score Coral
    controller.btn_RightBumper
        .whileTrue(Commands.deferredProxy(
            () -> subStateMachine.tryState(RobotState.SCORING_CORAL)))
        .onFalse(Commands.deferredProxy(
            () -> subStateMachine.tryState(RobotState.NONE)));

    // LB: Climb
    controller.btn_LeftBumper
        .whileTrue(Commands.deferredProxy(
            () -> subStateMachine.tryState(RobotState.CLIMBING_DEEP)))
        .onFalse(Commands.deferredProxy(
            () -> subStateMachine.tryState(RobotState.NONE)));

    // btn_East: Set Elevator to Neutral
    controller.btn_East
        .onTrue(Commands.runOnce(() -> subElevator.setNeutral(), subElevator));

    // btn_South: Prep Processor
    controller.btn_South
        .onTrue(Commands.deferredProxy(
            () -> subStateMachine.tryState(RobotState.PREP_PROCESSOR)));

    // btn_West: Clean L3 Reef
    controller.btn_West
        .whileTrue(Commands.deferredProxy(
            () -> subStateMachine.tryState(RobotState.CLEANING_L3)))
        .onFalse(Commands.deferredProxy(
            () -> subStateMachine.tryState(RobotState.NONE)));

    // btn_North: Clean L2 Reef
    controller.btn_North
        .whileTrue(Commands.deferredProxy(
            () -> subStateMachine.tryState(RobotState.CLEANING_L2)))
        .onFalse(Commands.deferredProxy(
            () -> subStateMachine.tryState(RobotState.NONE)));

    // btn_NorthWest: Prep Net
    controller.btn_NorthWest
        .onTrue(Commands.deferredProxy(
            () -> subStateMachine.tryState(RobotState.PREP_NET)));

    // btn_SouthEast: Eject Algae
    controller.btn_SouthEast
        .whileTrue(Commands.deferredProxy(
            () -> subStateMachine.tryState(RobotState.EJECTING_ALGAE)))
        .onFalse(Commands.deferredProxy(
            () -> subStateMachine.tryState(RobotState.NONE)));

    // btn_A/B/Y/X: Set Elevator to Coral Levels
    controller.btn_A
        .onTrue(Commands.deferredProxy(
            () -> subStateMachine.tryState(RobotState.PREP_CORAL_L1)));
    controller.btn_B
        .onTrue(Commands.deferredProxy(
            () -> subStateMachine.tryState(RobotState.PREP_CORAL_L2)));
    controller.btn_Y
        .onTrue(Commands.deferredProxy(
            () -> subStateMachine.tryState(RobotState.PREP_CORAL_L3)));
    controller.btn_X
        .onTrue(Commands.deferredProxy(
            () -> subStateMachine.tryState(RobotState.PREP_CORAL_L4)));

    // hasCoralTrigger
    hasCoralTrigger
        .whileTrue(Commands.deferredProxy(
            () -> subStateMachine.tryState(RobotState.HAS_CORAL)));

    // hasAlgaeTrigger
    hasAlgaeTrigger
        .whileTrue(Commands.deferredProxy(
            () -> subStateMachine.tryState(RobotState.HAS_ALGAE)));
  }

  public Command getAutonomousCommand() {
    // return new PathPlannerAuto("4-Piece-Low");
    return new PathPlannerAuto("L");
  }

  public static Command AddVisionMeasurement() {
    return new AddVisionMeasurement(subDrivetrain, subVision)
        .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming).ignoringDisable(true);
  }
}
