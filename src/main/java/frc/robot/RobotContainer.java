// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.List;
import java.util.Set;
import java.util.function.BooleanSupplier;

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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
import frc.robot.Constants.constCoralOuttake;
import frc.robot.Constants.constDrivetrain;
import frc.robot.Constants.constElevator;
import frc.robot.Constants.constField;
import frc.robot.Constants.constLED;
import frc.robot.Constants.constVision;
import frc.robot.Constants.*;
import frc.robot.RobotMap.mapControllers;
import frc.robot.commands.AddVisionMeasurement;
import frc.robot.commands.CoralStuckSoftwareLimitToggle;
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

  Boolean onRed = constField.ALLIANCE.isPresent() && constField.ALLIANCE.get().equals(Alliance.Red);

  private static DigitalInput isPracticeBot = new DigitalInput(RobotMap.PRAC_BOT_DIO);

  private final SN_XboxController conDriver = new SN_XboxController(mapControllers.DRIVER_USB);
  private final SN_XboxController conOperator = new SN_XboxController(mapControllers.OPERATOR_USB);

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
  private final CoralStuckSoftwareLimitToggle comCoralStuckSoftwareLimit = new CoralStuckSoftwareLimitToggle(
      subElevator);

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

  private final Trigger coralMovedBack = new Trigger(
      () -> subCoralOuttake.getDesiredOuttakeSpeed() == 0
          && subCoralOuttake.sensorSeesCoral());

  Command HAS_CORAL_RUMBLE = new HasGamePieceRumble(conDriver, conOperator, RumbleType.kRightRumble,
      Constants.constControllers.HAS_CORAL_RUMBLE_INTENSITY);
  Command READY_TO_LEAVE_RUMBLE = new HasGamePieceRumble(conDriver, conOperator, RumbleType.kRightRumble,
      Constants.constControllers.READY_TO_LEAVE_INTENSITY);

  Command HAS_ALGAE_RUMBLE = new HasGamePieceRumble(conDriver, conOperator, RumbleType.kLeftRumble,
      Constants.constControllers.HAS_ALGAE_RUMBLE_INTENSITY);

  public static boolean justScored = false;
  public static final Trigger justScoredTrigger = new Trigger(() -> justScored);
  private final Trigger readyToLiftElevator = new Trigger(() -> subDrivetrain.isAlignedCoral());
  private final Trigger readyToLiftNet = new Trigger(
      () -> subDrivetrain.isAlignedNet() && subStateMachine.getDriverState().equals(DriverState.NET_AUTO_DRIVING));

  private final Trigger readyToPlaceCoral = new Trigger(() -> subElevator.isAtAnyCoralScoringPosition()
      && subDrivetrain.isAlignedCoral());

  Pair<RobotState, Pose2d>[] SELECTED_AUTO_PREP_MAP;
  public static String SELECTED_AUTO_PREP_MAP_NAME = "none :("; // For logging :p
  int AUTO_PREP_NUM = 0;

  public RobotContainer() {
    RobotController.setBrownoutVoltage(5.5);
    conDriver.setLeftDeadband(constControllers.DRIVER_LEFT_STICK_DEADBAND);
    zeroSubsystems.addRequirements(subStateMachine);
    manualZeroSubsystems.addRequirements(subStateMachine);

    subDrivetrain
        .setDefaultCommand(
            new DriveManual(subStateMachine, subDrivetrain, subElevator, subAlgaeIntake, conDriver.axis_LeftY,
                conDriver.axis_LeftX,
                conDriver.axis_RightX, conDriver.btn_RightBumper, conDriver.btn_LeftTrigger, conDriver.btn_RightTrigger,
                conDriver.btn_B,
                conDriver.btn_X, conDriver.btn_East, conDriver.btn_LeftBumper, conDriver.btn_Start));

    configureDriverBindings(conDriver);
    configureOperatorBindings(conOperator);
    configureSensorBindings();
    configureAutoBindings();
    configureAutoSelector();

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
    // controller.btn_Back.onTrue(Commands.runOnce(() ->
    // subDrivetrain.resetModulesToAbsolute()));
    controller.btn_Back.onTrue(comCoralStuckSoftwareLimit);

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
            Commands
                .runOnce(
                    () -> subDrivetrain
                        .resetPoseToPose(Constants.constField.getAllFieldPositions(onRed, true).get()[0])));

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
        .whileTrue(TRY_EJECTING_CORAL_WITH_ALGAE)
        .onFalse(TRY_NONE)
        .onFalse(TRY_HAS_ALGAE);

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

    coralMovedBack
        .onTrue(Commands.sequence(
            Commands.runOnce(() -> subCoralOuttake.setCoralOuttakeSpeed(constCoralOuttake.CORAL_OUTTAKE_SPEED_SLOW)),
            Commands.waitUntil(() -> !subCoralOuttake.sensorSeesCoral()),
            Commands.runOnce(() -> subCoralOuttake.setCoralOuttakeSpeed(0))));

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

    readyToLiftNet.onTrue(Commands.runOnce(
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
    return subDrivetrain.isAlignedCoral();
  }

  public boolean coralStuckSoftwareLimitEnable() {
    return comCoralStuckSoftwareLimit.isCoralStuck();
  }

  /**
   * @return If the robot is the practice robot
   */
  public static boolean isPracticeBot() {
    return !isPracticeBot.get();
  }

  public void resetClimbBool() {
    subClimber.setClimberPreped(false);
  }

  public void checkForCoral() {
    if (subCoralOuttake.sensorSeesCoral()) {
      subStateMachine.setRobotState(RobotState.HAS_CORAL);
      subCoralOuttake.setHasCoral(true);
    }
  }

  // ------ Autos ------
  public Command getAutonomousCommand() {
    AUTO_PREP_NUM = 0;
    selectAutoMap();
    return Commands.runOnce(() -> subHopper.runHopper(0.1)).andThen(autoChooser.getSelected());
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
        DriverState.REEF_AUTO_DRIVING, subStateMachine, false, false)).repeatedly();

    Command algaeAutoAlign = Commands
        .runOnce(() -> subDrivetrain.algaeAutonomousPeriodAlign(SELECTED_AUTO_PREP_MAP[AUTO_PREP_NUM].getSecond(),
            MetersPerSecond.of(0),
            MetersPerSecond.of(0), DegreesPerSecond.of(0), 1.0, false,
            constDrivetrain.TELEOP_AUTO_ALIGN.MAX_AUTO_DRIVE_REEF_DISTANCE, DriverState.ALGAE_AUTO_DRIVING,
            DriverState.ALGAE_AUTO_DRIVING, subStateMachine, false, false))
        .repeatedly();

    Command netAutoAlign = Commands.runOnce(() -> subDrivetrain.autoPeriodNetAlign(subStateMachine)).repeatedly();

    NamedCommands.registerCommand("PlaceSequence",
        Commands.sequence(
            Commands.deadline(
                driveAutoAlign.asProxy().until(() -> subDrivetrain.isAlignedCoral()).withTimeout(1),
                TRY_PREP_CORAL_L4.asProxy()),
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
        Commands.runOnce(() -> subStateMachine.setRobotState(RobotState.HAS_CORAL))
            .alongWith(Commands.runOnce(() -> subCoralOuttake.setHasCoral(true))
                .alongWith(Commands.runOnce(() -> subAlgaeIntake.setAlgaePivotAngle(constAlgaeIntake.MAX_POS)))));

    NamedCommands.registerCommand("CleanL2Reef",
        Commands.sequence(
            algaeAutoAlign.asProxy().until(() -> subDrivetrain.isAlignedAlgae()).alongWith(TRY_CLEANING_L2.asProxy())
                .until(() -> subStateMachine.getRobotState() == RobotState.HAS_ALGAE),
            Commands.runOnce(() -> subAlgaeIntake.setHasAlgaeOverride(true))).asProxy());

    NamedCommands.registerCommand("CleanL3Reef",
        Commands.sequence(
            algaeAutoAlign.asProxy().until(() -> subDrivetrain.isAlignedAlgae()).alongWith(TRY_CLEANING_L3.asProxy())
                .until(() -> subStateMachine.getRobotState() == RobotState.HAS_ALGAE),
            Commands.runOnce(() -> subAlgaeIntake.setHasAlgaeOverride(true))).asProxy());

    NamedCommands.registerCommand("PrepPlaceWithAlgae",
        Commands.runOnce(() -> subStateMachine.tryState(RobotState.PREP_CORAL_L4_WITH_ALGAE))
            .until(() -> subStateMachine.getRobotState() == RobotState.PREP_CORAL_L4_WITH_ALGAE)
            .asProxy());

    NamedCommands.registerCommand("PrepNet",
        Commands.deadline(
            netAutoAlign.asProxy().until(() -> subDrivetrain.isAlignedNet()).withTimeout(1), TRY_PREP_NET.asProxy()));
    NamedCommands.registerCommand("ScoreAlgaeSequence", Commands.sequence(
        Commands.waitUntil(
            () -> subElevator.isAtSetPointWithTolerance(constElevator.ALGAE_PREP_NET, constElevator.NET_TOLERANCE)),
        TRY_SCORING_ALGAE.asProxy().withTimeout(0.45),
        TRY_NONE.asProxy().withTimeout(0.01),
        TRY_CLEANING_L2.asProxy().withTimeout(0.01),
        Commands.runOnce(() -> AUTO_PREP_NUM++)));

    NamedCommands.registerCommand("PrepProcessor",
        TRY_PREP_PROCESSOR.asProxy());

    // -- Event Markers --
    EventTrigger prepL2 = new EventTrigger("PrepL2");
    prepL2
        .onTrue(new DeferredCommand(() -> subStateMachine.tryState(RobotState.PREP_CORAL_L1),
            Set.of(subStateMachine)).repeatedly()
            .until(() -> subStateMachine.getRobotState() == RobotState.PREP_CORAL_L1));

    EventTrigger prepPlace = new EventTrigger("PrepPlace");
    prepPlace
        .onTrue(new DeferredCommand(() -> subStateMachine.tryState(RobotState.PREP_CORAL_L4),
            Set.of(subStateMachine)).repeatedly()
            .until(() -> subStateMachine.getRobotState() == RobotState.PREP_CORAL_L4));

    EventTrigger getCoralStationPiece = new EventTrigger("GetCoralStationPiece");
    getCoralStationPiece.onTrue(new DeferredCommand(() -> subStateMachine.tryState(RobotState.INTAKING_CORAL),
        Set.of(subStateMachine)));

    EventTrigger prepClean = new EventTrigger("PrepCleanL2");
    prepClean
        .onTrue(new DeferredCommand(() -> subStateMachine.tryState(RobotState.CLEANING_L2),
            Set.of(subStateMachine)).repeatedly()
            .until(() -> subStateMachine.getRobotState() == RobotState.CLEANING_L2));
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
    List<Pose2d> fieldPositions = constField.getReefPositionsClose(constField.isRedAlliance()).get();
    List<Pose2d> algaePositions = constField.getAlgaePositions(constField.isRedAlliance()).get();

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
        fourPieceLow[2] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, fieldPositions.get(1)); // B
        fourPieceLow[3] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, fieldPositions.get(4)); // E
        return fourPieceLow;
      case "Four_Piece_High_Single_Tickle":
        Pair<RobotState, Pose2d>[] fourPieceHighSingleTickle = new Pair[4];
        fourPieceHighSingleTickle[0] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, fieldPositions.get(11)); // L
        fourPieceHighSingleTickle[1] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, fieldPositions.get(10)); // K
        fourPieceHighSingleTickle[2] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, fieldPositions.get(0)); // A
        fourPieceHighSingleTickle[3] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, fieldPositions.get(9)); // J
        return fourPieceHighSingleTickle;
      case "Algae_Net":
        Pair<RobotState, Pose2d>[] algaeNet = new Pair[3];
        algaeNet[0] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, fieldPositions.get(6)); // G
        algaeNet[1] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, algaePositions.get(3)); // ALGAE GH
        algaeNet[2] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, algaePositions.get(4)); // ALGAE IJ
        return algaeNet;
      case "Algae_Mid_Net":
        Pair<RobotState, Pose2d>[] algaeMidNet = new Pair[4];
        algaeMidNet[0] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, fieldPositions.get(6)); // G
        algaeMidNet[1] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, algaePositions.get(3)); // ALGAE GH
        algaeMidNet[2] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, algaePositions.get(4)); // ALGAE IJ
        algaeMidNet[3] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, algaePositions.get(2)); // ALGAE EF
        return algaeMidNet;
      case "Algae_Near_Net":
        Pair<RobotState, Pose2d>[] algaeNearNet = new Pair[4];
        algaeNearNet[0] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, fieldPositions.get(7)); // H
        algaeNearNet[1] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, algaePositions.get(3)); // ALGAE GH
        algaeNearNet[2] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, algaePositions.get(4)); // ALGAE IJ
        algaeNearNet[3] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, algaePositions.get(5)); // ALGAE KL
        return algaeNearNet;
      case "Algae_Far_Net":
        Pair<RobotState, Pose2d>[] algaeFarNet = new Pair[4];
        algaeFarNet[0] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, fieldPositions.get(6)); // G
        algaeFarNet[1] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, algaePositions.get(3)); // ALGAE GH
        algaeFarNet[2] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, algaePositions.get(2)); // ALGAE EF
        algaeFarNet[3] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, algaePositions.get(1)); // ALGAE CD
        return algaeFarNet;
      case "Right_Algae_Net":
        Pair<RobotState, Pose2d>[] rightAlgaeNet = new Pair[3];
        rightAlgaeNet[0] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, fieldPositions.get(6)); // G
        rightAlgaeNet[1] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, algaePositions.get(3)); // ALGAE GH
        rightAlgaeNet[2] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, algaePositions.get(2)); // ALGAE EF
        return rightAlgaeNet;
      case "Moo_High":
        Pair<RobotState, Pose2d>[] mooHigh = new Pair[4];
        mooHigh[0] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, fieldPositions.get(9)); // j
        mooHigh[1] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, fieldPositions.get(11)); // l
        mooHigh[2] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, fieldPositions.get(10)); // k
        mooHigh[3] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, fieldPositions.get(8)); // i
        return mooHigh;
      case "Moo_Low":
        Pair<RobotState, Pose2d>[] mooLow = new Pair[4];
        mooLow[0] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, fieldPositions.get(4)); // E
        mooLow[1] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, fieldPositions.get(2)); // C
        mooLow[2] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, fieldPositions.get(3)); // D
        mooLow[3] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, fieldPositions.get(5)); // F
        return mooLow;
      default:
        Pair<RobotState, Pose2d>[] noAutoSelected = new Pair[1];
        noAutoSelected[0] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, new Pose2d());
        return noAutoSelected;
    }
  }
}