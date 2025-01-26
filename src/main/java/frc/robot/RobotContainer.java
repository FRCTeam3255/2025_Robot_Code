// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.frcteam3255.joystick.SN_XboxController;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.units.Units;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.*;
import frc.robot.RobotMap.mapControllers;
import frc.robot.commands.states.*;
import frc.robot.commands.*;
import frc.robot.commands.Zeroing.ManualZeroElevator;
import frc.robot.commands.Zeroing.ZeroElevator;
import frc.robot.subsystems.*;
import frc.robot.subsystems.StateMachine.RobotState;

@Logged
public class RobotContainer {
  private static DigitalInput isPracticeBot = new DigitalInput(RobotMap.PRAC_BOT_DIO);

  private final SN_XboxController conDriver = new SN_XboxController(mapControllers.DRIVER_USB);
  private final SN_XboxController conOperator = new SN_XboxController(mapControllers.OPERATOR_USB);
  private final SN_XboxController conTester = new SN_XboxController(mapControllers.TESTER_USB);

  private final Drivetrain subDrivetrain = new Drivetrain();
  private final Hopper subHopper = new Hopper();
  private static final Vision subVision = new Vision();
  private final AlgaeIntake subAlgaeIntake = new AlgaeIntake();
  private final CoralOuttake subCoralOuttake = new CoralOuttake();
  private final Climber subClimber = new Climber();
  private final Elevator subElevator = new Elevator();
  private final LED subLED = new LED();
  private final StateMachine subStateMachine = new StateMachine(subAlgaeIntake, subClimber, subCoralOuttake,
      subDrivetrain, subElevator, subHopper, subLED);

  private final IntakeCoralHopper comIntakeCoralHopper = new IntakeCoralHopper(subStateMachine, subHopper,
      subCoralOuttake, subLED);
  private final Climb comClimb = new Climb(subStateMachine, subClimber, subLED);
  private final PlaceCoral comPlaceCoral = new PlaceCoral(subStateMachine,
      subCoralOuttake, subLED);
  private final ScoringAlgae comScoringAlgae = new ScoringAlgae(subStateMachine, subAlgaeIntake, subLED);
  private final PrepProcessor comPrepProcessor = new PrepProcessor(subStateMachine, subElevator, subAlgaeIntake,
      subLED);
  private final PrepNet comPrepNet = new PrepNet(subStateMachine, subElevator, subAlgaeIntake, subLED);
  private final CleaningL3Reef comCleaningL3Reef = new CleaningL3Reef(subStateMachine, subElevator, subAlgaeIntake,
      subLED);
  private final CleaningL2Reef comCleaningL2Reef = new CleaningL2Reef(subStateMachine, subElevator, subAlgaeIntake,
      subLED);
  private final IntakingAlgaeGround comIntakingAlgaeGround = new IntakingAlgaeGround(subStateMachine, subElevator,
      subAlgaeIntake, subLED);
  private final EjectingAlgae comEjectingAlgae = new EjectingAlgae(subStateMachine, subAlgaeIntake, subLED);

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

  Command TRY_PREP_ALGAE_0 = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.PREP_ALGAE_ZERO));

  Command TRY_PREP_CORAL_0 = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.PREP_CORAL_ZERO));

  private final Trigger hasCoralTrigger = new Trigger(subCoralOuttake::hasCoral);
  private final Trigger hasAlgaeTrigger = new Trigger(subAlgaeIntake::hasAlgae);

  private Pose3d elevatorStageOne = Pose3d.kZero;
  private Pose3d elevatorCarriage = Pose3d.kZero;
  private Pose3d algaeIntake = Pose3d.kZero;

  public RobotContainer() {
    conDriver.setLeftDeadband(constControllers.DRIVER_LEFT_STICK_DEADBAND);

    subDrivetrain
        .setDefaultCommand(
            new DriveManual(subDrivetrain, conDriver.axis_LeftY, conDriver.axis_LeftX, conDriver.axis_RightX,
                conDriver.btn_LeftBumper, conDriver.btn_LeftTrigger, conDriver.btn_RightTrigger));

    configureDriverBindings(conDriver);
    configureOperatorBindings(conOperator);
    configureSensorBindings();
    configureAutoBindings();
    configureAutoSelector();
    configureTesterBindings(conTester);

    subDrivetrain.resetModulesToAbsolute();

    if (subCoralOuttake.hasCoral()) {
      subStateMachine.setRobotState(RobotState.HAS_CORAL);
    }
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
    NamedCommands.registerCommand("PrepPlace",
        Commands.sequence(TRY_PREP_CORAL_L3.asProxy()));

    NamedCommands.registerCommand("PlaceSequence",
        Commands.sequence(
            TRY_SCORING_CORAL.asProxy().until(() -> !hasCoralTrigger.getAsBoolean()),
            Commands.waitSeconds(1.5),
            TRY_NONE.asProxy().until(() -> !hasCoralTrigger.getAsBoolean())));

    NamedCommands.registerCommand("PrepCoralStation",
        Commands.print("Prep Coral Station"));

    NamedCommands.registerCommand("GetCoralStationPiece",
        Commands.sequence(
            TRY_INTAKING_CORAL_HOPPER.asProxy().until(hasCoralTrigger),
            TRY_PREP_CORAL_L3.asProxy()));
  }

  private void configureDriverBindings(SN_XboxController controller) {
    controller.btn_B
        .onTrue(TRY_CLIMBING_DEEP);

    controller.btn_North
        .onTrue(Commands.runOnce(() -> subDrivetrain.resetModulesToAbsolute()));
  }

  private void configureOperatorBindings(SN_XboxController controller) {
    controller.btn_LeftTrigger
        .whileTrue(TRY_INTAKING_CORAL_HOPPER)
        .onFalse(TRY_NONE);

    controller.btn_RightTrigger
        .whileTrue(TRY_SCORING_CORAL)
        .onFalse(TRY_NONE);

    controller.btn_LeftBumper
        .whileTrue(TRY_INTAKING_ALGAE_GROUND)
        .onFalse(TRY_NONE);

    controller.btn_RightBumper
        .whileTrue(TRY_SCORING_ALGAE)
        .onFalse(TRY_NONE);

    // TODO: Has Coral Overide Back BTN
    // TODO: Has Algae Overide Meneu BTN

    controller.btn_North
        .onTrue(TRY_PREP_NET);

    controller.btn_East
        .whileTrue(TRY_CLEANING_L3)
        .onFalse(TRY_NONE);

    controller.btn_West
        .whileTrue(TRY_CLEANING_L2)
        .onFalse(TRY_NONE);

    controller.btn_South
        .whileTrue(TRY_PREP_PROCESSOR);

    controller.btn_A
        .onTrue(TRY_PREP_CORAL_L1);

    controller.btn_B
        .onTrue(TRY_PREP_CORAL_L3);

    controller.btn_X
        .onTrue(TRY_PREP_CORAL_L2);

    controller.btn_Y
        .onTrue(TRY_PREP_CORAL_L4);

    controller.btn_LeftStick
        .onTrue(TRY_PREP_ALGAE_0);

    controller.btn_RightStick
        .onTrue(TRY_PREP_CORAL_0);
  }

  private void configureSensorBindings() {
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

  public Command checkForManualZeroing() {
    return new ManualZeroElevator(subElevator).ignoringDisable(true);
  }

  /**
   * Returns the command to zero all subsystems. This will make all subsystems
   * move
   * themselves downwards until they see a current spike and cancel any incoming
   * commands that
   * require those motors. If the zeroing does not end within a certain time
   * frame (set in constants), it will interrupt itself.
   * 
   * @return Parallel commands to zero the Climber, Elevator, and Shooter Pivot
   */
  public Command zeroSubsystems() {
    Command returnedCommand = new ParallelCommandGroup(
        new ZeroElevator(subElevator).withTimeout(constElevator.ZEROING_TIMEOUT.in(Units.Seconds)),
        new ZeroAlgaeIntake(subAlgaeIntake).withTimeout(constAlgaeIntake.ZEROING_TIMEOUT.in(Units.Seconds)))
        .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming);
    returnedCommand.addRequirements(subStateMachine);
    return returnedCommand;
  }

  public Command AddVisionMeasurement() {
    return new AddVisionMeasurement(subDrivetrain, subVision)
        .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming).ignoringDisable(true);
  }

  /**
   * @return If the robot is the practice robot
   */
  public static boolean isPracticeBot() {
    return !isPracticeBot.get();
  }

  public void updateLoggedPoses() {
    double elevatorPos, algaeAngle;

    // If we're in simulation, we can't log real mechanism data because they don't
    // exist. Instead, we'll log where we *want* the mechanisms to be and assume
    // they get there instantly.
    if (Robot.isSimulation()) {
      elevatorPos = subElevator.getLastDesiredPosition().in(Units.Meters) / 2;
      algaeAngle = subAlgaeIntake.getLastDesiredPivotAngle().in(Units.Degrees);
    } else {
      // Use real positions
      elevatorPos = (subElevator.getElevatorPosition().in(Units.Meters) / 2);
      algaeAngle = subAlgaeIntake.getPivotAngle().in(Units.Degrees);
    }

    elevatorStageOne = new Pose3d(new Translation3d(0.0889,
        0,
        0.109474 + elevatorPos), Rotation3d.kZero);

    elevatorCarriage = elevatorStageOne
        .transformBy(new Transform3d(new Translation3d(0, 0, Units.Inches.of(1).in(Units.Meters) + elevatorPos),
            Rotation3d.kZero));

    algaeIntake = elevatorCarriage
        .transformBy(new Transform3d(new Translation3d(0.075438, 0, 0.292354), new Rotation3d(0, algaeAngle, 0)));
  }
}
