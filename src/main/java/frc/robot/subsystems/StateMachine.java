// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;

import com.frcteam3255.joystick.SN_XboxController;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import frc.robot.Constants.constClimber;
import frc.robot.Constants.constElevator;
import frc.robot.Elastic;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.states.None;
import frc.robot.commands.states.climbing.ClimberDeploying;
import frc.robot.commands.states.climbing.ClimberRetracting;
import frc.robot.commands.states.climbing.ManualClimberDeploying;
import frc.robot.commands.states.first_scoring_element.CleaningL2Reef;
import frc.robot.commands.states.first_scoring_element.CleaningL3Reef;
import frc.robot.commands.states.first_scoring_element.EjectCoral;
import frc.robot.commands.states.first_scoring_element.IndexingCoral;
import frc.robot.commands.states.first_scoring_element.IntakeCoralHopper;
import frc.robot.commands.states.first_scoring_element.IntakingAlgaeGround;
import frc.robot.commands.states.hold_scoring_elements.HasAlgae;
import frc.robot.commands.states.hold_scoring_elements.HasCoral;
import frc.robot.commands.states.hold_scoring_elements.HasCoralAndAlgae;
import frc.robot.commands.states.prep_algae.PrepAlgaeZero;
import frc.robot.commands.states.prep_algae.PrepAlgaeZeroWithCoral;
import frc.robot.commands.states.prep_algae.PrepNet;
import frc.robot.commands.states.prep_algae.PrepNetWithCoral;
import frc.robot.commands.states.prep_algae.PrepProcessor;
import frc.robot.commands.states.prep_algae.PrepProcessorWithCoral;
import frc.robot.commands.states.prep_coral.PrepCoralLv;
import frc.robot.commands.states.prep_coral.PrepCoralLvWithAlgae;
import frc.robot.commands.states.prep_coral.PrepCoralZero;
import frc.robot.commands.states.prep_coral.PrepCoralZeroWithAlgae;
import frc.robot.commands.states.scoring.ScoringAlgae;
import frc.robot.commands.states.scoring.ScoringAlgaeWithCoral;
import frc.robot.commands.states.scoring.ScoringCoral;
import frc.robot.commands.states.scoring.ScoringCoralWithAlgae;
import frc.robot.commands.states.second_scoring_element.CleaningL2ReefWithCoral;
import frc.robot.commands.states.second_scoring_element.CleaningL3ReefWithCoral;
import frc.robot.commands.states.second_scoring_element.EjectCoralWithAlgae;
import frc.robot.commands.states.second_scoring_element.IndexingCoralWithAlgae;
import frc.robot.commands.states.second_scoring_element.IntakeCoralWithAlgae;

@Logged
public class StateMachine extends SubsystemBase {
  public static DriverState currentDriverState;
  public static RobotState currentRobotState;
  @NotLogged
  AlgaeIntake subAlgaeIntake;
  @NotLogged
  Climber subClimber;
  @NotLogged
  CoralOuttake subCoralOuttake;
  @NotLogged
  Drivetrain subDrivetrain;
  @NotLogged
  Elevator subElevator;
  @NotLogged
  Hopper subHopper;
  @NotLogged
  LED subLED;
  @NotLogged
  SN_XboxController conOperator;
  @NotLogged
  StateMachine subStateMachine = this;

  /** Creates a new StateMachine. */
  public StateMachine(AlgaeIntake subAlgaeIntake, Climber subClimber, CoralOuttake subCoralOuttake,
      Drivetrain subDrivetrain, Elevator subElevator, Hopper subHopper, LED subLED, SN_XboxController conOperator) {
    currentRobotState = RobotState.NONE;
    currentDriverState = DriverState.MANUAL;

    this.subAlgaeIntake = subAlgaeIntake;
    this.subClimber = subClimber;
    this.subCoralOuttake = subCoralOuttake;
    this.subDrivetrain = subDrivetrain;
    this.subElevator = subElevator;
    this.subHopper = subHopper;
    this.subLED = subLED;
    this.conOperator = conOperator;
  }

  public void setDriverState(DriverState driverState) {
    currentDriverState = driverState;
  }

  public void setRobotState(RobotState robotState) {
    currentRobotState = robotState;
  }

  public DriverState getDriverState() {
    return currentDriverState;
  }

  public RobotState getRobotState() {
    return currentRobotState;
  }

  public Command tryCoralOverride() {
    RobotState[] goToHasBothStates = { RobotState.HAS_ALGAE, RobotState.PREP_PROCESSOR, RobotState.PREP_NET,
        RobotState.PREP_PROCESSOR, RobotState.INTAKING_CORAL_WITH_ALGAE, RobotState.EJECTING_CORAL_WITH_ALGAE,
        RobotState.INDEXING_CORAL_WITH_ALGAE };

    subCoralOuttake.setHasCoral(true);
    for (RobotState state : goToHasBothStates) {
      if (currentRobotState == state) {
        return new HasCoralAndAlgae(subStateMachine, subCoralOuttake, subLED, subAlgaeIntake, subElevator, subHopper);
      }
    }
    return new HasCoral(subStateMachine, subCoralOuttake, subLED, subAlgaeIntake, subElevator, subHopper);
  }

  public boolean inCleaningState() {
    RobotState[] goToHasBothStates = { RobotState.CLEANING_L2, RobotState.CLEANING_L3,
        RobotState.CLEANING_L2_WITH_CORAL,
        RobotState.CLEANING_L3_WITH_CORAL };

    for (RobotState state : goToHasBothStates) {
      if (currentRobotState == state) {
        return true;
      }
    }
    return false;
  }

  public boolean inPrepState() {
    RobotState[] prepStates = { RobotState.PREP_CORAL_L1, RobotState.PREP_CORAL_L2, RobotState.PREP_CORAL_L3,
        RobotState.PREP_CORAL_L4, RobotState.PREP_CORAL_ZERO, RobotState.PREP_CORAL_L1_WITH_ALGAE,
        RobotState.PREP_CORAL_L2_WITH_ALGAE,
        RobotState.PREP_CORAL_L3_WITH_ALGAE, RobotState.PREP_CORAL_L4_WITH_ALGAE,
        RobotState.PREP_CORAL_ZERO_WITH_ALGAE };

    for (RobotState state : prepStates) {
      if (currentRobotState == state) {
        return true;
      }
    }
    return false;
  }

  public boolean inAlgaeWithCoralState() {
    RobotState[] AlgaeWithCoralStates = { RobotState.PREP_CORAL_L1_WITH_ALGAE,
        RobotState.PREP_CORAL_L2_WITH_ALGAE,
        RobotState.PREP_CORAL_L3_WITH_ALGAE, RobotState.PREP_CORAL_L4_WITH_ALGAE,
        RobotState.PREP_CORAL_ZERO_WITH_ALGAE };

    for (RobotState state : AlgaeWithCoralStates) {
      if (currentRobotState == state) {
        return true;
      }
    }
    return false;
  }

  public Command tryState(RobotState desiredState) {
    // The most elegant solution of all time... nested switch statements :)
    // haters please refer to this:
    // https://www.chiefdelphi.com/t/frc-3255-supernurds-2025-build-thread/477499/104

    // working!

    switch (desiredState) {
      case NONE:
        switch (currentRobotState) {
          case INTAKING_CORAL:
          case EJECTING_CORAL:
          case INTAKING_ALGAE_GROUND:
          case CLEANING_L2:
          case CLEANING_L3:
          case SCORING_CORAL:
          case SCORING_ALGAE:
            return new None(subStateMachine, subCoralOuttake, subHopper, subAlgaeIntake, subClimber, subElevator,
                subLED);
          case CLIMBER_DEPLOYING:
          case CLIMBER_RETRACTING:
            if (subClimber.isClimberPreped()) {
              return Commands.print("Climber is prepped!!! Not safe to return to NONE >____<");
            } else {
              Elastic.selectTab("Teleoperated");
              return new None(subStateMachine, subCoralOuttake, subHopper, subAlgaeIntake, subClimber, subElevator,
                  subLED);
            }
          case MANUAL_CLIMBER_DEPLOYING:
            if (subClimber.getClimberPosition().lte(constClimber.VALID_NONE_STATE_THRESHOLD)) {
              Elastic.selectTab("Teleoperated");
              return new None(subStateMachine, subCoralOuttake, subHopper, subAlgaeIntake, subClimber, subElevator,
                  subLED);
            } else {
              return Commands.print("Climber is too far down!!! Not safe to return to NONE >____<");
            }
        }
        break;

      // --- Manip. 1 scoring element ---
      // ------------ Algae -------------
      case INTAKING_ALGAE_GROUND:
        switch (currentRobotState) {
          case NONE:
          case HAS_ALGAE:
            return new IntakingAlgaeGround(subStateMachine, subElevator, subAlgaeIntake, subLED);
        }
        break;

      case CLEANING_L2:
        switch (currentRobotState) {
          case NONE:
          case CLEANING_L3:
            return Commands.either(new CleaningL2Reef(subStateMachine, subElevator, subAlgaeIntake, subLED),
                Commands.print("CORAL IS STUCK IN ELEVATOR!!!! :("), subCoralOuttake.isSafeToMoveElevator());
        }
        break;

      case CLEANING_L3:
        switch (currentRobotState) {
          case NONE:
          case CLEANING_L2:
            return Commands.either(new CleaningL3Reef(subStateMachine, subElevator, subAlgaeIntake, subLED),
                Commands.print("CORAL IS STUCK IN ELEVATOR!!!! :("), subCoralOuttake.isSafeToMoveElevator());
        }
        break;

      // ------------ Coral -------------
      case INTAKING_CORAL:
        switch (currentRobotState) {
          case NONE:
            return new IntakeCoralHopper(subStateMachine, subHopper, subCoralOuttake, subLED, subElevator,
                subAlgaeIntake);
        }
        break;

      case EJECTING_CORAL:
        switch (currentRobotState) {
          case NONE:
          case HAS_CORAL:
          case INTAKING_CORAL:
          case INDEXING_CORAL:
            return new EjectCoral(subStateMachine, subCoralOuttake, subLED, subHopper, subElevator);
        }
        break;

      case INDEXING_CORAL:
        switch (currentRobotState) {
          case INTAKING_CORAL:
            return new IndexingCoral(subStateMachine, subHopper, subCoralOuttake, subAlgaeIntake);
        }
        break;

      // --- Manip. 2nd scoring element ---
      // ------------ Algae -------------
      case CLEANING_L2_WITH_CORAL:
        switch (currentRobotState) {
          case HAS_CORAL:
          case HAS_CORAL_AND_ALGAE:
          case CLEANING_L3_WITH_CORAL:
          case PREP_CORAL_L1:
          case PREP_CORAL_L2:
          case PREP_CORAL_L3:
          case PREP_CORAL_L4:
          case PREP_CORAL_ZERO:
            return Commands.either(new CleaningL2ReefWithCoral(subStateMachine, subElevator, subAlgaeIntake, subLED),
                Commands.print("CORAL IS STUCK IN ELEVATOR!!!! :("), subCoralOuttake.isSafeToMoveElevator());
        }
        break;

      case CLEANING_L3_WITH_CORAL:
        switch (currentRobotState) {
          case HAS_CORAL:
          case HAS_CORAL_AND_ALGAE:
          case CLEANING_L2_WITH_CORAL:
          case PREP_CORAL_L1:
          case PREP_CORAL_L2:
          case PREP_CORAL_L3:
          case PREP_CORAL_L4:
          case PREP_CORAL_ZERO:
            return Commands.either(new CleaningL3ReefWithCoral(subStateMachine, subElevator, subAlgaeIntake, subLED),
                Commands.print("CORAL IS STUCK IN ELEVATOR!!!! :("), subCoralOuttake.isSafeToMoveElevator());
        }
        break;

      case INTAKING_CORAL_WITH_ALGAE:
        switch (currentRobotState) {
          case HAS_ALGAE:
          case PREP_PROCESSOR:
          case PREP_NET:
          case PREP_ALGAE_ZERO:
            return new IntakeCoralWithAlgae(subStateMachine, subHopper, subCoralOuttake, subLED, subElevator,
                subAlgaeIntake);
        }
        break;

      case EJECTING_CORAL_WITH_ALGAE:
        switch (currentRobotState) {
          case HAS_CORAL_AND_ALGAE:
          case INTAKING_CORAL_WITH_ALGAE:
          case INDEXING_CORAL_WITH_ALGAE:
          case HAS_ALGAE:
            return new EjectCoralWithAlgae(subStateMachine, subCoralOuttake, subLED, subHopper, subElevator);
        }
        break;

      case INDEXING_CORAL_WITH_ALGAE:
        switch (currentRobotState) {
          case INTAKING_CORAL_WITH_ALGAE:
            return new IndexingCoralWithAlgae(subStateMachine, subHopper, subCoralOuttake, subAlgaeIntake);
        }
        break;

      // --- Hold 1 Scoring ELement ---
      case HAS_CORAL:
        switch (currentRobotState) {
          case INDEXING_CORAL:
          case CLEANING_L2_WITH_CORAL:
          case CLEANING_L3_WITH_CORAL:
          case SCORING_ALGAE_WITH_CORAL:
            return new HasCoral(subStateMachine, subCoralOuttake, subLED, subAlgaeIntake, subElevator, subHopper);
        }
        break;

      case HAS_ALGAE:
        switch (currentRobotState) {
          case INTAKING_ALGAE_GROUND:
          case CLEANING_L2:
          case CLEANING_L3:
          case SCORING_CORAL_WITH_ALGAE:
          case INTAKING_CORAL_WITH_ALGAE:
          case EJECTING_CORAL_WITH_ALGAE:
            return new HasAlgae(subStateMachine, subAlgaeIntake, subLED, subCoralOuttake, subHopper, subElevator);
        }
        break;

      // --- Hold 2nd Scoring ELement ---
      case HAS_CORAL_AND_ALGAE:
        switch (currentRobotState) {
          case INDEXING_CORAL_WITH_ALGAE:
          case CLEANING_L2_WITH_CORAL:
          case CLEANING_L3_WITH_CORAL:
            return new HasCoralAndAlgae(subStateMachine, subCoralOuttake, subLED, subAlgaeIntake, subElevator,
                subHopper);
        }
        break;

      // -- Prep Coral Only --
      case PREP_CORAL_L1:
        switch (currentRobotState) {
          case HAS_CORAL:
          case PREP_CORAL_L2:
          case PREP_CORAL_L3:
          case PREP_CORAL_L4:
          case PREP_CORAL_ZERO:
            return Commands.either(
                new PrepCoralLv(subStateMachine, subElevator, subAlgaeIntake, constElevator.CORAL_L1_HEIGHT, subLED),
                Commands.print("CORAL IS STUCK IN ELEVATOR!!!! :("), subCoralOuttake.isSafeToMoveElevator());
        }
        break;

      case PREP_CORAL_L2:
        switch (currentRobotState) {
          case HAS_CORAL:
          case PREP_CORAL_L1:
          case PREP_CORAL_L3:
          case PREP_CORAL_L4:
          case PREP_CORAL_ZERO:
            return Commands.either(
                new PrepCoralLv(subStateMachine, subElevator, subAlgaeIntake, constElevator.CORAL_L2_HEIGHT, subLED),
                Commands.print("CORAL IS STUCK IN ELEVATOR!!!! :("), subCoralOuttake.isSafeToMoveElevator());
        }
        break;

      case PREP_CORAL_L3:
        switch (currentRobotState) {
          case HAS_CORAL:
          case PREP_CORAL_L1:
          case PREP_CORAL_L2:
          case PREP_CORAL_L4:
          case PREP_CORAL_ZERO:
            return Commands.either(
                new PrepCoralLv(subStateMachine, subElevator, subAlgaeIntake, constElevator.CORAL_L3_HEIGHT, subLED),
                Commands.print("CORAL IS STUCK IN ELEVATOR!!!! :("), subCoralOuttake.isSafeToMoveElevator());
        }
        break;

      case PREP_CORAL_L4:
        switch (currentRobotState) {
          case HAS_CORAL:
          case PREP_CORAL_L1:
          case PREP_CORAL_L2:
          case PREP_CORAL_L3:
          case PREP_CORAL_ZERO:
            return Commands.either(
                new PrepCoralLv(subStateMachine, subElevator, subAlgaeIntake, constElevator.CORAL_L4_HEIGHT, subLED),
                Commands.print("CORAL IS STUCK IN ELEVATOR!!!! :("), subCoralOuttake.isSafeToMoveElevator());
        }
        break;

      case PREP_CORAL_ZERO:
        switch (currentRobotState) {
          case PREP_CORAL_L1:
          case PREP_CORAL_L2:
          case PREP_CORAL_L3:
          case PREP_CORAL_L4:
          case HAS_CORAL:
            return Commands.either(new PrepCoralZero(subStateMachine, subElevator, subLED),
                Commands.print("CORAL IS STUCK IN ELEVATOR!!!! :("), subCoralOuttake.isSafeToMoveElevator());
        }
        break;

      // -- Prep Algae Only --
      case PREP_NET:
        switch (currentRobotState) {
          case HAS_ALGAE:
          case PREP_ALGAE_ZERO:
          case PREP_PROCESSOR:
            return Commands.either(new PrepNet(subStateMachine, subElevator, subAlgaeIntake, subLED),
                Commands.print("CORAL IS STUCK IN ELEVATOR!!!! :("), subCoralOuttake.isSafeToMoveElevator());
        }
        break;

      case PREP_PROCESSOR:
        switch (currentRobotState) {
          case HAS_ALGAE:
          case PREP_ALGAE_ZERO:
          case PREP_NET:
            return Commands.either(new PrepProcessor(subStateMachine, subElevator, subAlgaeIntake, subLED),
                Commands.print("CORAL IS STUCK IN ELEVATOR!!!! :("), subCoralOuttake.isSafeToMoveElevator());
        }
        break;

      case PREP_ALGAE_ZERO:
        switch (currentRobotState) {
          case HAS_ALGAE:
          case PREP_NET:
          case PREP_PROCESSOR:
            return Commands.either(new PrepAlgaeZero(subStateMachine, subElevator, subAlgaeIntake, subLED),
                Commands.print("CORAL IS STUCK IN ELEVATOR!!!! :("), subCoralOuttake.isSafeToMoveElevator());
        }
        break;

      // -- Prep Coral with Algae --
      case PREP_CORAL_L1_WITH_ALGAE:
        switch (currentRobotState) {
          case HAS_CORAL_AND_ALGAE:
          case PREP_CORAL_L2_WITH_ALGAE:
          case PREP_CORAL_L3_WITH_ALGAE:
          case PREP_CORAL_L4_WITH_ALGAE:
          case PREP_CORAL_ZERO_WITH_ALGAE:
          case PREP_PROCESSOR_WITH_CORAL:
          case PREP_NET_WITH_CORAL:
          case PREP_ALGAE_ZERO_WITH_CORAL:
            return Commands.either(
                new PrepCoralLvWithAlgae(subStateMachine, subElevator, constElevator.CORAL_L1_HEIGHT, subLED,
                    subAlgaeIntake),
                Commands.print("CORAL IS STUCK IN ELEVATOR!!!! :("), subCoralOuttake.isSafeToMoveElevator());

        }
        break;

      case PREP_CORAL_L2_WITH_ALGAE:
        switch (currentRobotState) {
          case HAS_CORAL_AND_ALGAE:
          case PREP_CORAL_L1_WITH_ALGAE:
          case PREP_CORAL_L3_WITH_ALGAE:
          case PREP_CORAL_L4_WITH_ALGAE:
          case PREP_CORAL_ZERO_WITH_ALGAE:
          case PREP_PROCESSOR_WITH_CORAL:
          case PREP_NET_WITH_CORAL:
          case PREP_ALGAE_ZERO_WITH_CORAL:
            return Commands.either(
                new PrepCoralLvWithAlgae(subStateMachine, subElevator, constElevator.CORAL_L2_HEIGHT, subLED,
                    subAlgaeIntake),
                Commands.print("CORAL IS STUCK IN ELEVATOR!!!! :("), subCoralOuttake.isSafeToMoveElevator());

        }
        break;

      case PREP_CORAL_L3_WITH_ALGAE:
        switch (currentRobotState) {
          case HAS_CORAL_AND_ALGAE:
          case PREP_CORAL_L1_WITH_ALGAE:
          case PREP_CORAL_L2_WITH_ALGAE:
          case PREP_CORAL_L4_WITH_ALGAE:
          case PREP_CORAL_ZERO_WITH_ALGAE:
          case PREP_PROCESSOR_WITH_CORAL:
          case PREP_NET_WITH_CORAL:
          case PREP_ALGAE_ZERO_WITH_CORAL:
            return Commands.either(
                new PrepCoralLvWithAlgae(subStateMachine, subElevator, constElevator.CORAL_L3_HEIGHT, subLED,
                    subAlgaeIntake),
                Commands.print("CORAL IS STUCK IN ELEVATOR!!!! :("), subCoralOuttake.isSafeToMoveElevator());
        }
        break;

      case PREP_CORAL_L4_WITH_ALGAE:
        switch (currentRobotState) {
          case HAS_CORAL_AND_ALGAE:
          case PREP_CORAL_L1_WITH_ALGAE:
          case PREP_CORAL_L2_WITH_ALGAE:
          case PREP_CORAL_L3_WITH_ALGAE:
          case PREP_CORAL_ZERO_WITH_ALGAE:
          case PREP_PROCESSOR_WITH_CORAL:
          case PREP_NET_WITH_CORAL:
          case PREP_ALGAE_ZERO_WITH_CORAL:
            return Commands.either(
                new PrepCoralLvWithAlgae(subStateMachine, subElevator, constElevator.CORAL_L4_HEIGHT, subLED,
                    subAlgaeIntake),
                Commands.print("CORAL IS STUCK IN ELEVATOR!!!! :("), subCoralOuttake.isSafeToMoveElevator());
        }
        break;

      case PREP_CORAL_ZERO_WITH_ALGAE:
        switch (currentRobotState) {
          case HAS_CORAL_AND_ALGAE:
          case PREP_CORAL_L1_WITH_ALGAE:
          case PREP_CORAL_L2_WITH_ALGAE:
          case PREP_CORAL_L3_WITH_ALGAE:
          case PREP_CORAL_L4_WITH_ALGAE:
          case PREP_PROCESSOR_WITH_CORAL:
          case PREP_NET_WITH_CORAL:
          case PREP_ALGAE_ZERO_WITH_CORAL:
            return Commands.either(new PrepCoralZeroWithAlgae(subStateMachine, subElevator, subLED),
                Commands.print("CORAL IS STUCK IN ELEVATOR!!!! :("), subCoralOuttake.isSafeToMoveElevator());

        }
        break;

      // -- Prep Algae with Coral --
      case PREP_NET_WITH_CORAL:
        switch (currentRobotState) {
          case HAS_CORAL_AND_ALGAE:
          case PREP_CORAL_L1_WITH_ALGAE:
          case PREP_CORAL_L2_WITH_ALGAE:
          case PREP_CORAL_L3_WITH_ALGAE:
          case PREP_CORAL_L4_WITH_ALGAE:
          case PREP_CORAL_ZERO_WITH_ALGAE:
          case PREP_PROCESSOR_WITH_CORAL:
          case PREP_ALGAE_ZERO_WITH_CORAL:
            return Commands.either(new PrepNetWithCoral(subStateMachine, subElevator, subAlgaeIntake, subLED),
                Commands.print("CORAL IS STUCK IN ELEVATOR!!!! :("), subCoralOuttake.isSafeToMoveElevator());
        }
        break;

      case PREP_PROCESSOR_WITH_CORAL:
        switch (currentRobotState) {
          case HAS_CORAL_AND_ALGAE:
          case PREP_CORAL_L1_WITH_ALGAE:
          case PREP_CORAL_L2_WITH_ALGAE:
          case PREP_CORAL_L3_WITH_ALGAE:
          case PREP_CORAL_L4_WITH_ALGAE:
          case PREP_CORAL_ZERO_WITH_ALGAE:
          case PREP_NET_WITH_CORAL:
          case PREP_ALGAE_ZERO_WITH_CORAL:
            return Commands.either(new PrepProcessorWithCoral(subStateMachine, subElevator, subAlgaeIntake, subLED),
                Commands.print("CORAL IS STUCK IN ELEVATOR!!!! :("), subCoralOuttake.isSafeToMoveElevator());
        }
        break;

      case PREP_ALGAE_ZERO_WITH_CORAL:
        switch (currentRobotState) {
          case HAS_CORAL_AND_ALGAE:
          case PREP_CORAL_L1_WITH_ALGAE:
          case PREP_CORAL_L2_WITH_ALGAE:
          case PREP_CORAL_L3_WITH_ALGAE:
          case PREP_CORAL_L4_WITH_ALGAE:
          case PREP_CORAL_ZERO_WITH_ALGAE:
          case PREP_NET_WITH_CORAL:
          case PREP_PROCESSOR_WITH_CORAL:
            return Commands.either(new PrepAlgaeZeroWithCoral(subStateMachine, subElevator, subAlgaeIntake, subLED),
                Commands.print("CORAL IS STUCK IN ELEVATOR!!!! :("), subCoralOuttake.isSafeToMoveElevator());
        }
        break;

      // -- Scoring --
      case SCORING_CORAL:
        switch (currentRobotState) {
          case PREP_CORAL_L1:
          case PREP_CORAL_L2:
          case PREP_CORAL_L3:
          case PREP_CORAL_L4:
          case PREP_CORAL_ZERO:
            return new ScoringCoral(subCoralOuttake, subStateMachine, subElevator, subLED, subAlgaeIntake, conOperator,
                getRobotState());
        }
        break;

      case SCORING_ALGAE:
        switch (currentRobotState) {
          case PREP_NET:
          case PREP_PROCESSOR:
          case PREP_ALGAE_ZERO:
            return new ScoringAlgae(subStateMachine, subAlgaeIntake, subLED, subElevator);
        }
        break;

      case SCORING_ALGAE_WITH_CORAL:
        switch (currentRobotState) {
          case PREP_PROCESSOR_WITH_CORAL:
          case PREP_NET_WITH_CORAL:
          case PREP_ALGAE_ZERO_WITH_CORAL:
            return new ScoringAlgaeWithCoral(subStateMachine, subAlgaeIntake, subLED, subElevator);
        }
        break;

      case SCORING_CORAL_WITH_ALGAE:
        switch (currentRobotState) {
          case PREP_CORAL_ZERO_WITH_ALGAE:
          case PREP_CORAL_L1_WITH_ALGAE:
          case PREP_CORAL_L2_WITH_ALGAE:
          case PREP_CORAL_L3_WITH_ALGAE:
          case PREP_CORAL_L4_WITH_ALGAE:
            return new ScoringCoralWithAlgae(subCoralOuttake, subStateMachine, subElevator, subDrivetrain, subLED,
                conOperator,
                getRobotState());
        }
        break;

      // -- Climbing --
      case CLIMBER_DEPLOYING:
        return new ClimberDeploying(subStateMachine, subClimber, subElevator, subAlgaeIntake, subLED);

      case CLIMBER_RETRACTING:
        switch (currentRobotState) {
          case NONE:
          case CLIMBER_DEPLOYING:
          case CLIMBER_RETRACTING:
          case MANUAL_CLIMBER_DEPLOYING:
            return new ClimberRetracting(subStateMachine, subClimber, subAlgaeIntake, subLED);
        }
        break;

      case MANUAL_CLIMBER_DEPLOYING:
        switch (currentRobotState) {
          case NONE:
          case CLIMBER_RETRACTING:
          case CLIMBER_DEPLOYING:
          case MANUAL_CLIMBER_DEPLOYING:
            return new ManualClimberDeploying(subStateMachine, subClimber, subElevator, subAlgaeIntake, subLED);
        }
        break;

    }
    return Commands
        .print("ITS SO OVER D: Invalid State Provided, Blame Eli. Attempted to go to: " + desiredState.toString()
            + " while at " + currentRobotState.toString());
  }

  public static enum DriverState {
    MANUAL,
    REEF_ROTATION_SNAPPING,
    CORAL_STATION_ROTATION_SNAPPING,
    REEF_AUTO_DRIVING,
    CORAL_STATION_AUTO_DRIVING,
    PROCESSOR_ROTATION_SNAPPING,
    PROCESSOR_AUTO_DRIVING,
    NET_ROTATION_SNAPPING,
    NET_AUTO_DRIVING,
    ALGAE_ROTATION_SNAPPING,
    ALGAE_AUTO_DRIVING,
    CAGE_ROTATION_SNAPPING
  }

  /**
   * Represents the various states the robot can be in during operation.
   * 
   * @see https://www.tldraw.com/ro/lFqVEhO80IajGo7JezZaz
   */
  public static enum RobotState {
    NONE,

    // Manip. 1 Scoring eLement
    CLEANING_L2,
    CLEANING_L3,
    INTAKING_ALGAE_GROUND,
    INTAKING_CORAL,
    INDEXING_CORAL,
    EJECTING_CORAL,

    // Manip. 2nd Scoring element
    CLEANING_L2_WITH_CORAL,
    CLEANING_L3_WITH_CORAL,
    EJECTING_CORAL_WITH_ALGAE,
    INDEXING_CORAL_WITH_ALGAE,
    INTAKING_CORAL_WITH_ALGAE,

    // Hold 1 Scoring element
    HAS_CORAL,
    HAS_ALGAE,

    // Hold 2 Scoring elements
    HAS_CORAL_AND_ALGAE,

    // Prep Coral Only
    PREP_CORAL_L1,
    PREP_CORAL_L2,
    PREP_CORAL_L3,
    PREP_CORAL_L4,
    PREP_CORAL_ZERO,

    // Prep Algae Only
    PREP_NET,
    PREP_PROCESSOR,
    PREP_ALGAE_ZERO,

    // Prep Coral with Algae
    PREP_CORAL_L1_WITH_ALGAE,
    PREP_CORAL_L2_WITH_ALGAE,
    PREP_CORAL_L3_WITH_ALGAE,
    PREP_CORAL_L4_WITH_ALGAE,
    PREP_CORAL_ZERO_WITH_ALGAE,

    // Prep Algae with Coral
    PREP_NET_WITH_CORAL,
    PREP_PROCESSOR_WITH_CORAL,
    PREP_ALGAE_ZERO_WITH_CORAL,

    // Scoring
    SCORING_CORAL,
    SCORING_ALGAE,
    SCORING_CORAL_WITH_ALGAE,
    SCORING_ALGAE_WITH_CORAL,

    // Climbing
    CLIMBER_DEPLOYING,
    CLIMBER_RETRACTING,
    MANUAL_CLIMBER_DEPLOYING,
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
