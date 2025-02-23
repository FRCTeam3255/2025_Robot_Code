// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.frcteam3255.joystick.SN_XboxController;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Set;

import edu.wpi.first.units.Units;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.*;
import frc.robot.RobotMap.mapControllers;
import frc.robot.commands.states.climbing.ClimberDeploying;
import frc.robot.commands.states.first_scoring_element.CleaningL2Reef;
import frc.robot.commands.states.first_scoring_element.CleaningL3Reef;
import frc.robot.commands.states.first_scoring_element.IntakeCoralHopper;
import frc.robot.commands.states.first_scoring_element.IntakingAlgaeGround;
import frc.robot.commands.states.prep_algae.PrepNet;
import frc.robot.commands.states.prep_algae.PrepProcessor;
import frc.robot.commands.states.scoring.ScoringAlgae;
import frc.robot.commands.states.scoring.ScoringCoral;
import frc.robot.commands.*;
import frc.robot.commands.Zeroing.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.StateMachine.DriverState;
import frc.robot.subsystems.StateMachine.RobotState;
import edu.wpi.first.wpilibj.RobotController;

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
  Command TRY_INTAKING_CORAL_HOPPER = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.INTAKING_CORAL));
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

  Command EJECTING_CORAL = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.EJECTING_CORAL));

  Command HAS_CORAL_OVERRIDE = Commands.runOnce(() -> subCoralOuttake.coralToggle());
  Command HAS_ALGAE_OVERRIDE = Commands.runOnce(() -> subAlgaeIntake.algaeToggle());

  Command zeroSubsystems = new ParallelCommandGroup(
      new ZeroElevator(subElevator).withTimeout(constElevator.ZEROING_TIMEOUT.in(Units.Seconds)),
      new ZeroAlgaeIntake(subAlgaeIntake).onlyIf(() -> !subAlgaeIntake.hasZeroed)
          .withTimeout(constAlgaeIntake.ZEROING_TIMEOUT.in(Units.Seconds)))
      .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming).withName("ZeroSubsystems");
  Command manualZeroSubsystems = new ManualZeroElevator(subElevator)
      .alongWith(new ManualZeroAlgaeIntake(subAlgaeIntake))
      .ignoringDisable(true).withName("ManualZeroSubsystems");

  private final Trigger hasCoralTrigger = new Trigger(() -> subCoralOuttake.hasCoral() && !subAlgaeIntake.hasAlgae());
  private final Trigger hasAlgaeTrigger = new Trigger(() -> !subCoralOuttake.hasCoral() && subAlgaeIntake.hasAlgae()
      && subStateMachine.getRobotState() != RobotState.SCORING_CORAL_WITH_ALGAE
      && subStateMachine.getRobotState() != RobotState.INTAKING_CORAL_WITH_ALGAE);
  private final Trigger hasBothTrigger = new Trigger(() -> subCoralOuttake.hasCoral() && subAlgaeIntake.hasAlgae());

  public RobotContainer() {
    RobotController.setBrownoutVoltage(5.5);
    conDriver.setLeftDeadband(constControllers.DRIVER_LEFT_STICK_DEADBAND);
    zeroSubsystems.addRequirements(subStateMachine);
    manualZeroSubsystems.addRequirements(subStateMachine);

    subDrivetrain
        .setDefaultCommand(
            new DriveManual(subStateMachine, subDrivetrain, subElevator, conDriver.axis_LeftY, conDriver.axis_LeftX,
                conDriver.axis_RightX,
                conDriver.btn_LeftBumper, conDriver.btn_LeftTrigger, conDriver.btn_RightTrigger, conDriver.btn_A,
                conDriver.btn_B, conDriver.btn_X, conDriver.btn_Y, conDriver.btn_South));

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

  private void configureAutoBindings() {
    // -- Named Commands --
    Command driveAutoAlign = Commands.runOnce(() -> subDrivetrain.autoAlign(Meters.of(0),
        subDrivetrain.getPose().nearest(constField.getReefPositions().get()), MetersPerSecond.of(0),
        MetersPerSecond.of(0), DegreesPerSecond.of(0), 1.0, false, Meters.of(1000), DriverState.REEF_AUTO_DRIVING,
        DriverState.REEF_AUTO_DRIVING, subStateMachine)).repeatedly();

    NamedCommands.registerCommand("PlaceSequence",
        Commands.sequence(
            driveAutoAlign.asProxy().until(() -> subDrivetrain
                .isAtPosition(subDrivetrain.getPose().nearest(constField.getReefPositions().get()))),
            Commands.runOnce(() -> subDrivetrain.drive(new ChassisSpeeds(), false)),
            TRY_SCORING_CORAL.asProxy()).until(() -> subStateMachine.getRobotState() == RobotState.NONE));

    NamedCommands.registerCommand("PrepPlace",
        Commands.sequence(
            Commands.runOnce(() -> subStateMachine.tryState(RobotState.PREP_CORAL_L4)).asProxy()));

    NamedCommands.registerCommand("GetCoralStationPiece",
        Commands.sequence(
            TRY_INTAKING_CORAL_HOPPER.asProxy().until(() -> subCoralOuttake.sensorSeesCoral()),
            TRY_NONE.asProxy().until(() -> subStateMachine.getRobotState() == RobotState.NONE))
            .withName("GetCoralStationPiece"));

    NamedCommands.registerCommand("ForceGamePiece",
        TRY_INTAKING_CORAL_HOPPER.asProxy().until(() -> subStateMachine.getRobotState() == RobotState.HAS_CORAL));

    // -- Event Markers --
    EventTrigger prepPlace = new EventTrigger("PrepPlace");
    prepPlace.onTrue(new DeferredCommand(() -> subStateMachine.tryState(RobotState.PREP_CORAL_L4),
        Set.of(subStateMachine)));
    EventTrigger getCoralStationPiece = new EventTrigger("GetCoralStationPiece");
    getCoralStationPiece.onTrue(new DeferredCommand(() -> subStateMachine.tryState(RobotState.INTAKING_CORAL),
        Set.of(subStateMachine)));
  }

  private void configureDriverBindings(SN_XboxController controller) {
    controller.btn_Y
        .whileTrue(TRY_CLIMBER_DEPLOYING);

    controller.btn_A
        .whileTrue(TRY_CLIMBER_RETRACTING);

    controller.btn_North
        .onTrue(Commands.runOnce(() -> subDrivetrain.resetPoseToPose(Pose2d.kZero)));
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

    controller.btn_Back.onTrue(HAS_CORAL_OVERRIDE);
    controller.btn_Start.onTrue(HAS_ALGAE_OVERRIDE);

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
        .whileTrue(EJECTING_CORAL)
        .onFalse(TRY_NONE);
  }

  private void configureSensorBindings() {
    hasCoralTrigger
        .whileTrue(TRY_HAS_CORAL);

    hasAlgaeTrigger
        .whileTrue(TRY_HAS_ALGAE);

    hasBothTrigger.whileTrue(TRY_HAS_CORAL_AND_ALGAE);
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

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public RobotState getRobotState() {
    return subStateMachine.getRobotState();
  }

  private void configureAutoSelector() {
    autoChooser = AutoBuilder.buildAutoChooser("4-Piece-Low");
    SmartDashboard.putData(autoChooser);
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
}