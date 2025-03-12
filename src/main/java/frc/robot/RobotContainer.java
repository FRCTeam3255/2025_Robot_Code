// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.List;
import java.util.Set;

import com.frcteam3255.joystick.SN_XboxController;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.constAlgaeIntake;
import frc.robot.Constants.constControllers;
import frc.robot.Constants.constElevator;
import frc.robot.Constants.constField;
import frc.robot.Constants.constLED;
import frc.robot.Constants.constVision;
import frc.robot.RobotMap.mapControllers;
import frc.robot.commands.AddVisionMeasurement;
import frc.robot.commands.DriveManual;
import frc.robot.commands.Zeroing.ManualZeroAlgaeIntake;
import frc.robot.commands.Zeroing.ManualZeroElevator;
import frc.robot.commands.Zeroing.ZeroAlgaeIntake;
import frc.robot.commands.Rumbles.HasGamePieceRumble;
import frc.robot.commands.Zeroing.ZeroElevator;
import frc.robot.commands.states.climbing.ClimberDeploying;
import frc.robot.commands.states.first_scoring_element.CleaningL2Reef;
import frc.robot.commands.states.first_scoring_element.CleaningL3Reef;
import frc.robot.commands.states.first_scoring_element.IntakeCoralHopper;
import frc.robot.commands.states.first_scoring_element.IntakingAlgaeGround;
import frc.robot.commands.states.prep_algae.PrepNet;
import frc.robot.commands.states.prep_algae.PrepProcessor;
import frc.robot.commands.states.scoring.ScoringAlgae;
import frc.robot.commands.states.scoring.ScoringCoral;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CoralOuttake;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.RobotPoses;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.StateMachine.DriverState;
import frc.robot.subsystems.StateMachine.RobotState;
import frc.robot.subsystems.Vision;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

@Logged
public class RobotContainer {
  private final String LEFT_LABEL = "Left";
  private final String RIGHT_LABEL = "Right";

  private static DigitalInput isPracticeBot = new DigitalInput(RobotMap.PRAC_BOT_DIO);

  private final SN_XboxController conDriver = new SN_XboxController(mapControllers.DRIVER_USB);
  private final SN_XboxController conOperator = new SN_XboxController(mapControllers.OPERATOR_USB);
  private final SN_XboxController conTester = new SN_XboxController(mapControllers.TESTER_USB);

  private final Drivetrain subDrivetrain = new Drivetrain();
  private final Hopper subHopper = new Hopper();
  private final Vision subVision = new Vision();
  private final AlgaeIntake subAlgaeIntake = new AlgaeIntake();
  private final CoralOuttake subCoralOuttake = new CoralOuttake();
  private final Climber subClimber = new Climber();
  private final Elevator subElevator = new Elevator();
  private final LED subLED = new LED();
  private final RobotPoses robotPose = new RobotPoses(subDrivetrain, subElevator, subAlgaeIntake, subCoralOuttake);
  private final StateMachine subStateMachine = new StateMachine(subAlgaeIntake, subClimber, subCoralOuttake,
      subDrivetrain, subElevator, subHopper, subLED, conOperator);
  private final IntakeCoralHopper comIntakeCoralHopper = new IntakeCoralHopper(subStateMachine, subHopper,
      subCoralOuttake, subLED, subElevator, subAlgaeIntake);
  private final ScoringAlgae comScoringAlgae = new ScoringAlgae(subStateMachine, subAlgaeIntake, subLED, subElevator);
  private final ClimberDeploying comClimb = new ClimberDeploying(subStateMachine, subClimber, subElevator,
      subAlgaeIntake, subLED);
  private final ScoringCoral comScoringCoral = new ScoringCoral(subCoralOuttake, subStateMachine, subElevator, subLED,
      subAlgaeIntake,
      conOperator, subStateMachine.getRobotState());
  private final PrepProcessor comPrepProcessor = new PrepProcessor(subStateMachine, subElevator, subAlgaeIntake,
      subLED);
  private final PrepNet comPrepNet = new PrepNet(subStateMachine, subElevator, subAlgaeIntake, subLED);
  private final CleaningL3Reef comCleaningL3Reef = new CleaningL3Reef(subStateMachine, subElevator, subAlgaeIntake,
      subLED);
  private final CleaningL2Reef comCleaningL2Reef = new CleaningL2Reef(subStateMachine, subElevator, subAlgaeIntake,
      subLED);
  private final IntakingAlgaeGround comIntakingAlgaeGround = new IntakingAlgaeGround(subStateMachine, subElevator,
      subAlgaeIntake, subLED);

  @NotLogged
  SendableChooser<Command> autoChooser = new SendableChooser<>();

  // -- STATES! --
  Command TRY_CLIMBER_DEPLOYING = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.CLIMBER_DEPLOYING));
  Command TRY_MANUAL_CLIMBER_DEPLOYING = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.MANUAL_CLIMBER_DEPLOYING));
  Command TRY_CLIMBER_RETRACTING = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.CLIMBER_RETRACTING));
  Command TRY_CLEANING_L3 = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.CLEANING_L3));
  Command TRY_CLEANING_L2 = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.CLEANING_L2));
  Command TRY_CLEANING_L3_WITH_CORAL = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.CLEANING_L3_WITH_CORAL));
  Command TRY_CLEANING_L2_WITH_CORAL = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.CLEANING_L2_WITH_CORAL));
  Command TRY_EJECTING_CORAL = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.EJECTING_CORAL));
  Command TRY_EJECTING_CORAL_WITH_ALGAE = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.EJECTING_CORAL_WITH_ALGAE));
  Command TRY_INTAKING_CORAL_HOPPER = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.INTAKING_CORAL));
  Command TRY_INDEXING_CORAL = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.INDEXING_CORAL));
  Command TRY_INDEXING_CORAL_WITH_ALGAE = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.INDEXING_CORAL_WITH_ALGAE));
  Command TRY_INTAKING_CORAL_WITH_ALGAE = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.INTAKING_CORAL_WITH_ALGAE));
  Command TRY_INTAKING_ALGAE_GROUND = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.INTAKING_ALGAE_GROUND));
  Command TRY_HAS_CORAL = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.HAS_CORAL));
  Command TRY_HAS_ALGAE = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.HAS_ALGAE));
  Command TRY_HAS_CORAL_AND_ALGAE = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.HAS_CORAL_AND_ALGAE));
  Command TRY_PREP_ALGAE_0 = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.PREP_ALGAE_ZERO));
  Command TRY_PREP_ALGAE_0_WITH_CORAL = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.PREP_ALGAE_ZERO_WITH_CORAL));
  Command TRY_PREP_NET = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.PREP_NET));
  Command TRY_PREP_NET_WITH_CORAL = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.PREP_NET_WITH_CORAL));
  Command TRY_PREP_PROCESSOR = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.PREP_PROCESSOR));
  Command TRY_PREP_PROCESSOR_WITH_CORAL = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.PREP_PROCESSOR_WITH_CORAL));
  Command TRY_PREP_CORAL_L1 = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.PREP_CORAL_L1));
  Command TRY_PREP_CORAL_L2 = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.PREP_CORAL_L2));
  Command TRY_PREP_CORAL_L3 = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.PREP_CORAL_L3));
  Command TRY_PREP_CORAL_L4 = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.PREP_CORAL_L4));
  Command TRY_PREP_CORAL_0 = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.PREP_CORAL_ZERO));
  Command TRY_PREP_CORAL_L1_WITH_ALGAE = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.PREP_CORAL_L1_WITH_ALGAE));
  Command TRY_PREP_CORAL_L2_WITH_ALGAE = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.PREP_CORAL_L2_WITH_ALGAE));
  Command TRY_PREP_CORAL_L3_WITH_ALGAE = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.PREP_CORAL_L3_WITH_ALGAE));
  Command TRY_PREP_CORAL_L4_WITH_ALGAE = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.PREP_CORAL_L4_WITH_ALGAE));
  Command TRY_PREP_CORAL_0_WITH_ALGAE = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.PREP_CORAL_ZERO_WITH_ALGAE));
  Command TRY_SCORING_ALGAE = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.SCORING_ALGAE));
  Command TRY_SCORING_CORAL = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.SCORING_CORAL));
  Command TRY_SCORING_ALGAE_WITH_CORAL = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.SCORING_ALGAE_WITH_CORAL));
  Command TRY_SCORING_CORAL_WITH_ALGAE = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.SCORING_CORAL_WITH_ALGAE));
  Command TRY_NONE = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.NONE));
  Command TRY_NONE_FROM_SCORING = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.NONE)
          .unless(() -> (subStateMachine.getRobotState() == RobotState.SCORING_CORAL
              || subStateMachine.getRobotState() == RobotState.SCORING_CORAL_WITH_ALGAE)));

  Command HAS_CORAL_OVERRIDE = Commands.deferredProxy(
      () -> subStateMachine.tryCoralOverride());

  Command HAS_ALGAE_OVERRIDE = Commands.runOnce(() -> subAlgaeIntake.algaeToggle());

  Command zeroSubsystems = new ParallelCommandGroup(
      new ZeroElevator(subElevator).withTimeout(constElevator.ZEROING_TIMEOUT.in(Units.Seconds)),
      new ZeroAlgaeIntake(subAlgaeIntake).onlyIf(() -> !subAlgaeIntake.hasZeroed)
          .withTimeout(constAlgaeIntake.ZEROING_TIMEOUT.in(Units.Seconds)))
      .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming).withName("ZeroSubsystems");
  Command manualZeroSubsystems = new ManualZeroElevator(subElevator, subLED)
      .alongWith(new ManualZeroAlgaeIntake(subAlgaeIntake, subLED))
      .ignoringDisable(true).withName("ManualZeroSubsystems");

  private final Trigger hasCoralTrigger = new Trigger(() -> subCoralOuttake.hasCoral() && !subAlgaeIntake.hasAlgae());
  private final Trigger indexingCoralTrigger = new Trigger(
      () -> subCoralOuttake.sensorSeesCoral() && (subStateMachine.getRobotState().equals(RobotState.INTAKING_CORAL)
          || subStateMachine.getRobotState().equals(RobotState.INTAKING_CORAL_WITH_ALGAE)));
  private final Trigger hasAlgaeTrigger = new Trigger(() -> !subCoralOuttake.hasCoral() && subAlgaeIntake.hasAlgae()
      && subStateMachine.getRobotState() != RobotState.SCORING_CORAL_WITH_ALGAE
      && subStateMachine.getRobotState() != RobotState.INTAKING_CORAL_WITH_ALGAE);
  private final Trigger hasBothTrigger = new Trigger(() -> subCoralOuttake.hasCoral() && subAlgaeIntake.hasAlgae());
  private final Trigger hasAlgaeStateTrigger = new Trigger(
      () -> subStateMachine.getRobotState() == RobotState.HAS_ALGAE);

  Command HAS_CORAL_RUMBLE = new HasGamePieceRumble(conDriver, conOperator, RumbleType.kRightRumble,
      Constants.constControllers.HAS_CORAL_RUMBLE_INTENSITY);
  Command READY_TO_LEAVE_RUMBLE = new HasGamePieceRumble(conDriver, conOperator, RumbleType.kRightRumble,
      Constants.constControllers.READY_TO_LEAVE_INTENSITY);

  Command HAS_ALGAE_RUMBLE = new HasGamePieceRumble(conDriver, conOperator, RumbleType.kLeftRumble,
      Constants.constControllers.HAS_ALGAE_RUMBLE_INTENSITY);

  public static boolean justScored = false;
  private final Trigger justScoredTrigger = new Trigger(() -> justScored);
  private final Trigger readyToLiftElevator = new Trigger(() -> subDrivetrain.isAligned());
  private final Trigger readyToPlaceCoral = new Trigger(() -> subElevator.isAtAnyCoralScoringPosition()
      && subDrivetrain.isAligned());

  Pair<RobotState, Pose2d>[] SELECTED_AUTO_PREP_MAP;
  String SELECTED_AUTO_PREP_MAP_NAME = "none :("; // For logging :p
  int AUTO_PREP_NUM = 0;

  public RobotContainer() {
    RobotController.setBrownoutVoltage(5.5);
    conDriver.setLeftDeadband(constControllers.DRIVER_LEFT_STICK_DEADBAND);
    zeroSubsystems.addRequirements(subStateMachine);
    manualZeroSubsystems.addRequirements(subStateMachine);

    subDrivetrain
        .setDefaultCommand(
            new DriveManual(subStateMachine, subDrivetrain, subElevator,
                conDriver.axis_LeftY, conDriver.axis_LeftX,
                conDriver.axis_RightX, conDriver.btn_RightBumper, conDriver.btn_LeftTrigger,
                conDriver.btn_RightTrigger,
                conDriver.btn_B,
                conDriver.btn_X, conDriver.btn_LeftBumper));

    configureDriverBindings(conDriver);
    configureOperatorBindings(conOperator);
    configureSensorBindings();
    configureAutoBindings();
    configureAutoSelector();
    configureTesterBindings(conTester);

    subDrivetrain.resetModulesToAbsolute();

    checkForCoral();
  }

  public void setMegaTag2(boolean setMegaTag2) {

    if (setMegaTag2) {
      subDrivetrain.swervePoseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(
          constVision.MEGA_TAG2_STD_DEVS_POSITION,
          constVision.MEGA_TAG2_STD_DEVS_POSITION,
          constVision.MEGA_TAG2_STD_DEVS_HEADING));
    } else {
      // Use MegaTag 1
      subDrivetrain.swervePoseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(
          constVision.MEGA_TAG1_STD_DEVS_POSITION,
          constVision.MEGA_TAG1_STD_DEVS_POSITION,
          constVision.MEGA_TAG1_STD_DEVS_HEADING));
    }
    subVision.setMegaTag2(setMegaTag2);
  }

  private void configureDriverBindings(SN_XboxController controller) {
    controller.btn_Start
        .onTrue(TRY_CLIMBER_DEPLOYING);

    controller.btn_Y
        .whileTrue(TRY_MANUAL_CLIMBER_DEPLOYING)
        .onFalse(TRY_NONE);

    controller.btn_A
        .whileTrue(TRY_CLIMBER_RETRACTING)
        .onFalse(TRY_NONE);

    controller.btn_North
        .onTrue(
            Commands.runOnce(() -> subDrivetrain.resetPoseToPose(Constants.constField.getFieldPositions().get()[0])));

    controller.btn_West.onTrue(Commands.sequence(Commands.runOnce(() -> subAlgaeIntake.hasZeroed = false),
        new ZeroAlgaeIntake(subAlgaeIntake)));
    controller.btn_South.onTrue(Commands.sequence(Commands.runOnce(() -> subElevator.hasZeroed = false),
        new ZeroElevator(subElevator)));

  }

  private void configureOperatorBindings(SN_XboxController controller) {
    // Intake Coral
    controller.btn_LeftTrigger
        .whileTrue(TRY_INTAKING_CORAL_HOPPER)
        .whileTrue(TRY_INTAKING_CORAL_WITH_ALGAE)
        .onFalse(TRY_NONE)
        .onFalse(TRY_HAS_ALGAE);

    // Score
    controller.btn_RightTrigger
        .onTrue(TRY_SCORING_CORAL)
        .onTrue(TRY_SCORING_ALGAE)
        .onTrue(TRY_SCORING_ALGAE_WITH_CORAL)
        .onTrue(TRY_SCORING_CORAL_WITH_ALGAE)
        .onFalse(TRY_NONE_FROM_SCORING)
        .onFalse(TRY_HAS_CORAL);

    // Intake Algae
    controller.btn_LeftBumper
        .whileTrue(TRY_INTAKING_ALGAE_GROUND)
        .onFalse(TRY_NONE);

    // Eject Coral
    controller.btn_RightBumper
        .whileTrue(TRY_EJECTING_CORAL)
        .onFalse(TRY_NONE);

    controller.btn_Start.onTrue(HAS_CORAL_OVERRIDE);
    controller.btn_Back.onTrue(HAS_ALGAE_OVERRIDE);

    // Net
    controller.btn_North
        .onTrue(TRY_PREP_NET)
        .onTrue(TRY_PREP_NET_WITH_CORAL);

    // L3
    controller.btn_East
        .whileTrue(TRY_CLEANING_L3)
        .whileTrue(TRY_CLEANING_L3_WITH_CORAL)
        .onFalse(TRY_HAS_CORAL)
        .onFalse(TRY_NONE);

    // L2
    controller.btn_West
        .whileTrue(TRY_CLEANING_L2)
        .whileTrue(TRY_CLEANING_L2_WITH_CORAL)
        .onFalse(TRY_HAS_CORAL)
        .onFalse(TRY_NONE);

    // Processor
    controller.btn_South
        .whileTrue(TRY_PREP_PROCESSOR_WITH_CORAL)
        .whileTrue(TRY_PREP_PROCESSOR);

    controller.btn_A
        .onTrue(TRY_PREP_CORAL_L1_WITH_ALGAE)
        .onTrue(TRY_PREP_CORAL_L1);

    controller.btn_B
        .onTrue(TRY_PREP_CORAL_L3_WITH_ALGAE)
        .onTrue(TRY_PREP_CORAL_L3);

    controller.btn_X
        .onTrue(TRY_PREP_CORAL_L2_WITH_ALGAE)
        .onTrue(TRY_PREP_CORAL_L2);

    controller.btn_Y
        .onTrue(TRY_PREP_CORAL_L4_WITH_ALGAE)
        .onTrue(TRY_PREP_CORAL_L4);

    controller.btn_LeftStick
        .onTrue(TRY_PREP_ALGAE_0_WITH_CORAL)
        .onTrue(TRY_PREP_ALGAE_0);

    controller.btn_RightStick
        .onTrue(TRY_PREP_CORAL_0_WITH_ALGAE)
        .onTrue(TRY_PREP_CORAL_0);

    controller.btn_RightBumper
        .whileTrue(TRY_EJECTING_CORAL)
        .onFalse(TRY_NONE);
  }

  private void configureSensorBindings() {
    indexingCoralTrigger.onTrue(HAS_CORAL_RUMBLE).onTrue(TRY_INDEXING_CORAL).onTrue(TRY_INDEXING_CORAL_WITH_ALGAE);

    hasAlgaeTrigger
        .whileTrue(TRY_HAS_ALGAE);

    hasCoralTrigger
        .whileTrue(TRY_HAS_CORAL);
    hasBothTrigger.whileTrue(TRY_HAS_CORAL_AND_ALGAE);
    hasAlgaeStateTrigger.onTrue(HAS_ALGAE_RUMBLE);

    readyToLiftElevator.onTrue(Commands.runOnce(
        () -> conOperator.setRumble(RumbleType.kLeftRumble, Constants.constControllers.READY_TO_RAISE_INTENSITY)))
        .onTrue(Commands.runOnce(
            () -> subLED.setLED(constLED.READY_TO_LIFT, 0)))
        .onFalse(Commands.runOnce(
            () -> conOperator.setRumble(RumbleType.kBothRumble, 0)));

    readyToPlaceCoral.onTrue(Commands.runOnce(
        () -> conOperator.setRumble(RumbleType.kBothRumble, Constants.constControllers.READY_TO_RAISE_INTENSITY)))
        .onTrue(Commands.runOnce(
            () -> subLED.setLED(constLED.READY_TO_PLACE, 0)))
        .onFalse(Commands.runOnce(
            () -> conOperator.setRumble(RumbleType.kBothRumble, 0)));

    justScoredTrigger.onTrue(READY_TO_LEAVE_RUMBLE).onTrue(Commands.runOnce(
        () -> subLED.setLED(constLED.READY_TO_LEAVE, 0)));
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
    controller.btn_RightTrigger
        .whileTrue(comScoringAlgae);

    // RB: Score Coral
    controller.btn_RightTrigger
        .onTrue(comScoringCoral);

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

  public RobotState getRobotState() {
    return subStateMachine.getRobotState();
  }

  public Command AddVisionMeasurement() {
    return new AddVisionMeasurement(subDrivetrain, subVision)
        .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming).ignoringDisable(true);
  }

  public boolean allZeroed() {
    return subElevator.hasZeroed && subAlgaeIntake.hasZeroed;
  }

  public boolean isAligned() {
    return subDrivetrain.isAligned();
  }

  public boolean elevatorAndAlgaeAtSetPoint() {
    return subElevator.isAtSetPoint() && subAlgaeIntake.isAtSetPoint();
  }

  /**
   * @return If the robot is the practice robot
   */
  public static boolean isPracticeBot() {
    return !isPracticeBot.get();
  }

  public void checkForCoral() {
    if (subCoralOuttake.sensorSeesCoral()) {
      subStateMachine.setRobotState(RobotState.HAS_CORAL);
      subCoralOuttake.setHasCoral(true);
    }
  }

  // ------ Autos ------
  public Command getAutonomousCommand() {
    selectAutoMap();
    return autoChooser.getSelected();
  }

  public void resetToAutoPose() {
    Rotation2d desiredRotation = Rotation2d.kZero;

    try {
      desiredRotation = PathPlannerAuto.getPathGroupFromAutoFile(autoChooser.getSelected().getName()).get(0)
          .getIdealStartingState().rotation();
      if (constField.isRedAlliance()) {
        desiredRotation = desiredRotation.plus(Rotation2d.k180deg);
      }
    } catch (Exception e) {
    }

    subDrivetrain.resetPoseToPose(new Pose2d(subDrivetrain.getPose().getTranslation(), desiredRotation));
  }

  private void configureAutoSelector() {
    autoChooser = AutoBuilder.buildAutoChooser("Four-Piece-Low");
    SmartDashboard.putData(autoChooser);
  }

  private void configureAutoBindings() {
    // i decorate my commands like a christmas tree
    // -- Named Commands --
    Command driveAutoAlign = Commands.runOnce(() -> subDrivetrain.autoAlign(Meters.of(0),
        SELECTED_AUTO_PREP_MAP[AUTO_PREP_NUM].getSecond(), MetersPerSecond.of(0),
        MetersPerSecond.of(0), DegreesPerSecond.of(0), 1.0, false, Meters.of(1000), DriverState.REEF_AUTO_DRIVING,
        DriverState.REEF_AUTO_DRIVING, subStateMachine)).repeatedly();

    NamedCommands.registerCommand("PlaceSequence",
        Commands.sequence(
            driveAutoAlign.asProxy().until(() -> subDrivetrain.isAligned()).withTimeout(1),
            Commands.runOnce(() -> subDrivetrain.drive(new ChassisSpeeds(), false)),
            TRY_PREP_CORAL_L4.asProxy().until(() -> subStateMachine.getRobotState() == RobotState.PREP_CORAL_L4),
            TRY_SCORING_CORAL.asProxy().until(() -> subStateMachine.getRobotState() == RobotState.NONE),
            Commands.runOnce(() -> AUTO_PREP_NUM++)).withName("PlaceSequence"));

    NamedCommands.registerCommand("PrepPlace",
        Commands.runOnce(() -> subStateMachine.tryState(RobotState.PREP_CORAL_L4))
            .until(() -> subStateMachine.getRobotState() == RobotState.PREP_CORAL_L4)
            .asProxy().withName("PrepPlace"));

    NamedCommands.registerCommand("GetCoralStationPiece",
        Commands.sequence(
            TRY_INTAKING_CORAL_HOPPER.asProxy().until(() -> subCoralOuttake.sensorSeesCoral()))
            .withName("GetCoralStationPiece"));

    NamedCommands.registerCommand("ForceGamePiece",
        Commands.either(
            Commands.runOnce(() -> subStateMachine.setRobotState(RobotState.HAS_CORAL))
                .alongWith(Commands.runOnce(() -> subCoralOuttake.setHasCoral(true))
                    .alongWith(Commands.runOnce(() -> subAlgaeIntake.setAlgaePivotAngle(constAlgaeIntake.MAX_POS)))),
            TRY_INTAKING_CORAL_HOPPER.asProxy().until(() -> subStateMachine.getRobotState() == RobotState.HAS_CORAL),
            subCoralOuttake.sensorSeesCoralSupplier()).withName("ForceGamePiece"));

    NamedCommands.registerCommand("CleanL2Reef",
        Commands.sequence(
            subStateMachine.tryState(RobotState.CLEANING_L2).asProxy().repeatedly()
                .until(() -> subStateMachine.getRobotState() == RobotState.HAS_ALGAE)
                .asProxy(),
            subStateMachine.tryState(RobotState.PREP_ALGAE_ZERO)
                .alongWith(Commands.runOnce(() -> subAlgaeIntake.setHasAlgaeOverride(true))).asProxy()));

    NamedCommands.registerCommand("CleanL3Reef",
        Commands.sequence(
            subStateMachine.tryState(RobotState.CLEANING_L3).asProxy().repeatedly()
                .until(() -> subStateMachine.getRobotState() == RobotState.HAS_ALGAE)
                .asProxy(),
            subStateMachine.tryState(RobotState.PREP_ALGAE_ZERO)
                .alongWith(Commands.runOnce(() -> subAlgaeIntake.setHasAlgaeOverride(true)))));

    NamedCommands.registerCommand("PrepPlaceWithAlgae",
        Commands.runOnce(() -> subStateMachine.tryState(RobotState.PREP_CORAL_L4_WITH_ALGAE))
            .until(() -> subStateMachine.getRobotState() == RobotState.PREP_CORAL_L4_WITH_ALGAE)
            .asProxy());

    NamedCommands.registerCommand("PrepNet",
        TRY_PREP_NET.asProxy()
            .until(() -> subStateMachine.getRobotState() == RobotState.PREP_NET));

    NamedCommands.registerCommand("ScoreAlgaeSequence", Commands.sequence(
        Commands.waitUntil(() -> subElevator.isAtSetPoint()),
        TRY_SCORING_ALGAE.asProxy().withTimeout(1),
        Commands.waitSeconds(0.5),
        TRY_NONE.asProxy().until(() -> subElevator.getElevatorPosition().lte(constElevator.INIT_TIP_HEIGHT))));

    // -- Event Markers --
    EventTrigger prepPlace = new EventTrigger("PrepPlace");
    prepPlace
        .onTrue(new DeferredCommand(() -> subStateMachine.tryState(RobotState.PREP_CORAL_L4),
            Set.of(subStateMachine)).repeatedly()
            .until(() -> subStateMachine.getRobotState() == RobotState.PREP_CORAL_L4));
    EventTrigger getCoralStationPiece = new EventTrigger("GetCoralStationPiece");
    getCoralStationPiece.onTrue(new DeferredCommand(() -> subStateMachine.tryState(RobotState.INTAKING_CORAL),
        Set.of(subStateMachine)));
  }

  /**
   * Populates the selected AutoMap for your autonomous command.
   */
  private void selectAutoMap() {
    SELECTED_AUTO_PREP_MAP = configureAutoPrepMaps(autoChooser.getSelected().getName());
    SELECTED_AUTO_PREP_MAP_NAME = autoChooser.getSelected().getName();
  }

  private Pair<RobotState, Pose2d>[] configureAutoPrepMaps(String selectedAuto) {
    RobotState AUTO_PREP_CORAL_4 = RobotState.PREP_CORAL_L4;
    RobotState AUTO_PREP_CORAL_2 = RobotState.PREP_CORAL_L2;
    List<Pose2d> fieldPositions = constField.getReefPositions().get();

    switch (selectedAuto) {
      case "Four_Piece_High":
        Pair<RobotState, Pose2d>[] fourPieceHigh = new Pair[4];
        fourPieceHigh[0] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, fieldPositions.get(11)); // L
        fourPieceHigh[1] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, fieldPositions.get(10)); // K
        fourPieceHigh[2] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, fieldPositions.get(0)); // A
        fourPieceHigh[3] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, fieldPositions.get(9)); // J
        return fourPieceHigh;
      case "Four_Piece_High_Double_Tickle":
        Pair<RobotState, Pose2d>[] fourPieceHighDoubleTickle = new Pair[4];
        fourPieceHighDoubleTickle[0] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, fieldPositions.get(11)); // L
        fourPieceHighDoubleTickle[1] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, fieldPositions.get(10)); // K
        fourPieceHighDoubleTickle[2] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, fieldPositions.get(0)); // A
        fourPieceHighDoubleTickle[3] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, fieldPositions.get(9)); // J
        return fourPieceHighDoubleTickle;
      case "Four_Piece_Low":
        Pair<RobotState, Pose2d>[] fourPieceLow = new Pair[4];
        fourPieceLow[0] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, fieldPositions.get(2)); // C
        fourPieceLow[1] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, fieldPositions.get(3)); // D
        fourPieceLow[2] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, fieldPositions.get(4)); // E
        fourPieceLow[3] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, fieldPositions.get(5)); // F
        return fourPieceLow;
      case "Four_Piece_High_Single_Tickle":
        Pair<RobotState, Pose2d>[] fourPieceHighSingleTickle = new Pair[4];
        fourPieceHighSingleTickle[0] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, fieldPositions.get(11)); // L
        fourPieceHighSingleTickle[1] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, fieldPositions.get(10)); // K
        fourPieceHighSingleTickle[2] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, fieldPositions.get(0)); // A
        fourPieceHighSingleTickle[3] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, fieldPositions.get(9)); // J
        return fourPieceHighSingleTickle;
      case "Algae_Net":
        Pair<RobotState, Pose2d>[] algaeNet = new Pair[1];
        algaeNet[0] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, fieldPositions.get(6)); // G
        return algaeNet;
      default:
        Pair<RobotState, Pose2d>[] noAutoSelected = new Pair[1];
        noAutoSelected[0] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, new Pose2d());
        return noAutoSelected;
    }
  }
}