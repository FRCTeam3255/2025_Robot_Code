// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import com.frcteam3255.joystick.SN_XboxController;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.units.Units;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

@Logged
public class RobotContainer {

  private final SN_XboxController conDriver = new SN_XboxController(mapControllers.DRIVER_USB);
  private final SN_XboxController conOperator = new SN_XboxController(mapControllers.OPERATOR_USB);
  private final SN_XboxController conTester = new SN_XboxController(mapControllers.TESTER_USB);

  private static final Drivetrain subDrivetrain = new Drivetrain();
  private final Hopper subHopper = new Hopper();
  private static final Vision subVision = new Vision();
  private final AlgaeIntake subAlgaeIntake = new AlgaeIntake();
  private final CoralOuttake subCoralOuttake = new CoralOuttake();
  private final Climber subClimber = new Climber();
  private final Elevator subElevator = new Elevator();
  private final StateMachine subStateMachine = new StateMachine(subAlgaeIntake, subClimber, subCoralOuttake,
      subDrivetrain, subElevator, subHopper);

  private final IntakeCoralHopper comIntakeCoralHopper = new IntakeCoralHopper(subStateMachine, subHopper,
      subCoralOuttake);
  private final Climb comClimb = new Climb(subStateMachine, subClimber);
  private final PlaceCoral comPlaceCoral = new PlaceCoral(subStateMachine,
      subCoralOuttake);
  private final ScoringAlgae comScoringAlgae = new ScoringAlgae(subStateMachine, subAlgaeIntake);
  private final PrepProcessor comPrepProcessor = new PrepProcessor(subStateMachine, subElevator);
  private final PrepNet comPrepNet = new PrepNet(subStateMachine, subElevator);
  private final CleaningL3Reef comCleaningL3Reef = new CleaningL3Reef(subStateMachine, subElevator, subAlgaeIntake);
  private final CleaningL2Reef comCleaningL2Reef = new CleaningL2Reef(subStateMachine, subElevator, subAlgaeIntake);
  private final IntakingAlgaeGround comIntakingAlgaeGround = new IntakingAlgaeGround(subStateMachine, subElevator,
      subAlgaeIntake);
  private final EjectingAlgae comEjectingAlgae = new EjectingAlgae(subStateMachine, subAlgaeIntake);

  Command TRY_INTAKING_CORAL_HOPPER = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.INTAKING_CORAL_HOPPER));

  Command TRY_NONE = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.NONE));

  Command TRY_INTAKING_ALGAE_GROUND = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.INTAKING_ALGAE_GROUND));

  Command TRY_EJECTING_ALGAE = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.EJECTING_ALGAE));

  Command TRY_SCORING_ALGAE = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.SCORING_ALGAE));

  Command TRY_SCORING_CORAL = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.SCORING_CORAL));

  Command TRY_CLIMBING_DEEP = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.CLIMBING_DEEP));

  Command TRY_PREP_PROCESSOR = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.PREP_PROCESSOR));

  Command TRY_CLEANING_L3 = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.CLEANING_L3));

  Command TRY_CLEANING_L2 = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.CLEANING_L2));

  Command TRY_PREP_NET = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.PREP_NET));

  Command TRY_PREP_CORAL_L1 = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.PREP_CORAL_L1));

  Command TRY_PREP_CORAL_L2 = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.PREP_CORAL_L2));

  Command TRY_PREP_CORAL_L3 = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.PREP_CORAL_L3));

  Command TRY_PREP_CORAL_L4 = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.PREP_CORAL_L4));

  Command TRY_HAS_CORAL = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.HAS_CORAL));

  Command TRY_HAS_ALGAE = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.HAS_ALGAE));

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
    configureAutoBindings();
    configureAutoSelector();
    configureTesterBindings(conTester);

    subDrivetrain.resetModulesToAbsolute();

  }

  private void configureAutoBindings() {
    NamedCommands.registerCommand("PrepPlace",
        Commands.sequence(
            Commands.deferredProxy(
                () -> subStateMachine.tryState(RobotState.PREP_CORAL_L3))));

    NamedCommands.registerCommand("PlaceSequence",
        Commands.sequence(
            Commands.deferredProxy(
                () -> subStateMachine.tryState(RobotState.SCORING_CORAL).until(() -> !hasCoralTrigger.getAsBoolean())),
            Commands.waitSeconds(1.5),
            Commands.deferredProxy(
                () -> subStateMachine.tryState(RobotState.NONE).until(() -> !hasCoralTrigger.getAsBoolean()))));

    NamedCommands.registerCommand("PrepCoralStation",
        Commands.print("Prep Coral Station"));

    NamedCommands.registerCommand("GetCoralStationPiece",
        Commands.sequence(
            Commands
                .deferredProxy(() -> subStateMachine.tryState(RobotState.INTAKING_CORAL_HOPPER).until(hasCoralTrigger)),
            Commands.deferredProxy(
                () -> subStateMachine.tryState(RobotState.PREP_CORAL_L3))));
  }

  private void configureDriverBindings(SN_XboxController controller) {
    controller.btn_B.onTrue(Commands.runOnce(() -> subDrivetrain.resetModulesToAbsolute()));
    controller.btn_Back
        .onTrue(Commands.runOnce(() -> subDrivetrain.resetPoseToPose(constField.getFieldPositions().get()[0])));
  }

  private void configureOperatorBindings(SN_XboxController controller) {

    // Start: Reset Elevator Sensor Position
    controller.btn_Start.onTrue(Commands.runOnce(() -> subElevator.resetSensorPosition(Units.Inches.of(0)))
        .ignoringDisable(true));

    controller.btn_Back
        .whileTrue(TRY_INTAKING_CORAL_HOPPER)
        .onFalse(TRY_NONE);

    controller.btn_LeftTrigger
        .whileTrue(TRY_INTAKING_ALGAE_GROUND)
        .onFalse(TRY_NONE);

    controller.btn_RightTrigger
        .onTrue(TRY_SCORING_ALGAE)
        .onFalse(TRY_NONE);

    controller.btn_RightBumper
        .whileTrue(TRY_SCORING_CORAL)
        .onFalse(TRY_NONE);

    controller.btn_LeftBumper
        .whileTrue(TRY_CLIMBING_DEEP)
        .onFalse(TRY_NONE);

    controller.btn_East
        .onTrue(Commands.runOnce(() -> subElevator.setNeutral(), subElevator));

    controller.btn_South
        .onTrue(TRY_PREP_PROCESSOR);

    controller.btn_West
        .whileTrue(TRY_CLEANING_L3)
        .onFalse(TRY_NONE);

    controller.btn_North
        .whileTrue(TRY_CLEANING_L2)
        .onFalse(TRY_NONE);

    controller.btn_NorthWest
        .onTrue(TRY_PREP_NET);

    // btn_SouthEast: Eject Algae
    controller.btn_SouthEast
        .whileTrue(TRY_EJECTING_ALGAE)
        .onFalse(TRY_NONE);

    controller.btn_A
        .onTrue(TRY_PREP_CORAL_L1);
    controller.btn_B
        .onTrue(TRY_PREP_CORAL_L2);
    controller.btn_Y
        .onTrue(TRY_PREP_CORAL_L3);
    controller.btn_X
        .onTrue(TRY_PREP_CORAL_L4);

    hasCoralTrigger
        .whileTrue(TRY_HAS_CORAL);

    hasAlgaeTrigger
        .whileTrue(TRY_HAS_ALGAE);
  }

  private void configureTesterBindings(SN_XboxController controller) {
    // Start: Reset Elevator Sensor Position
    controller.btn_Start.onTrue(Commands.runOnce(() -> subElevator.resetSensorPosition(Units.Inches.of(0)))
        .ignoringDisable(true));

    // Back: Intake Coral
    controller.btn_Back
        .whileTrue(comIntakeCoralHopper);

    // LT: Eat Algae
    controller.btn_LeftBumper
        .whileTrue(comIntakingAlgaeGround);

    // RT: Spit Algae
    controller.btn_RightBumper
        .whileTrue(comScoringAlgae);

    // RB: Score Coral
    controller.btn_RightTrigger
        .whileTrue(comPlaceCoral);

    // LB: Climb
    controller.btn_LeftTrigger
        .whileTrue(comClimb);

    // btn_East: Set Elevator to Neutral
    controller.btn_East
        .onTrue(Commands.runOnce(() -> subElevator.setNeutral(), subElevator));

    // btn_South: Prep Processor
    controller.btn_South
        .whileTrue(comPrepProcessor);

    // btn_West: Clean L3 Reef
    controller.btn_West
        .whileTrue(comCleaningL3Reef);

    // btn_North: Clean L2 Reef
    controller.btn_North
        .whileTrue(comCleaningL2Reef);

    // btn_NorthWest: Prep Net
    controller.btn_NorthWest
        .whileTrue(comPrepNet);

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
  
  SendableChooser<Command> autoChooser = new SendableChooser<>();
  
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  private void configureAutoSelector() {
    autoChooser.setDefaultOption("4-Piece-Low",
        Commands.sequence(Commands.runOnce(() -> subStateMachine.setRobotState(RobotState.HAS_CORAL)),
            new PathPlannerAuto("4-Piece-Low")));
    autoChooser.addOption("4-Piece-Low",
        Commands.sequence(Commands.runOnce(() -> subStateMachine.setRobotState(RobotState.HAS_CORAL)),
            new PathPlannerAuto("4-Piece-Low")));
    SmartDashboard.putData(autoChooser);
  }

  public static Command AddVisionMeasurement() {
    return new AddVisionMeasurement(subDrivetrain, subVision)
        .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming).ignoringDisable(true);
  }
}
