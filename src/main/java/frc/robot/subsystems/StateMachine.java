// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.frcteam3255.joystick.SN_XboxController;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import frc.robot.Constants.constElevator;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.states.*;

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

  public Command tryState(RobotState desiredState) {
    switch (desiredState) {
      case NONE: // UPDATED
        switch (currentRobotState) {
          case INTAKING_CORAL:
          case EJECTING_CORAL:
          case INTAKING_ALGAE_GROUND:
          case CLEANING_L2:
          case CLEANING_L3:
          case SCORING_CORAL:
          case SCORING_ALGAE:
          case CLIMBER_DEPLOYING:
          case CLIMBER_RETRACTING:
            return new None(subStateMachine, subCoralOuttake, subHopper, subAlgaeIntake, subClimber, subElevator,
                subLED);
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
            return new CleaningL2Reef(subStateMachine, subElevator, subAlgaeIntake, subLED);
        }
        break;

      case CLEANING_L3:
        switch (currentRobotState) {
          case NONE:
          case CLEANING_L2:
            return new CleaningL3Reef(subStateMachine, subElevator, subAlgaeIntake, subLED);
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
          case HAS_CORAL:
          case INTAKING_CORAL:
            return new EjectCoral(subStateMachine, subCoralOuttake, subLED);
        }
        break;

      // --- Manip. 2nd scoring element ---
      // ------------ Algae -------------
      case CLEANING_L2_WITH_CORAL:
        break;

      case CLEANING_L3_WITH_CORAL:
        break;

      case INTAKING_CORAL_WITH_ALGAE:
        break;

      // --- Hold 1 Scoring ELement ---
      case HAS_CORAL:
        switch (currentRobotState) {
          case INTAKING_CORAL:
            return new HasCoral(subStateMachine, subCoralOuttake, subLED);
        }
        break;

      case HAS_ALGAE:
        switch (currentRobotState) {
          case INTAKING_ALGAE_GROUND:
          case CLEANING_L2:
          case CLEANING_L3:
            return new HasAlgae(subStateMachine, subAlgaeIntake, subLED);
        }
        break;

      // --- Hold 2nd Scoring ELement ---
      case HAS_CORAL_AND_ALGAE:
        break;

      // -- Prep Coral Only --
      case PREP_CORAL_L1:
        switch (currentRobotState) {
          case HAS_CORAL:
          case PREP_CORAL_L2:
          case PREP_CORAL_L3:
          case PREP_CORAL_L4:
          case PREP_CORAL_ZERO:
            return new PrepCoralLv(subStateMachine, subElevator, constElevator.CORAL_L1_HEIGHT, subLED);
        }
        break;

      case PREP_CORAL_L2:
        switch (currentRobotState) {
          case HAS_CORAL:
          case PREP_CORAL_L1:
          case PREP_CORAL_L3:
          case PREP_CORAL_L4:
          case PREP_CORAL_ZERO:
            return new PrepCoralLv(subStateMachine, subElevator, constElevator.CORAL_L2_HEIGHT, subLED);
        }
        break;

      case PREP_CORAL_L3:
        switch (currentRobotState) {
          case HAS_CORAL:
          case PREP_CORAL_L1:
          case PREP_CORAL_L2:
          case PREP_CORAL_L4:
          case PREP_CORAL_ZERO:
            return new PrepCoralLv(subStateMachine, subElevator, constElevator.CORAL_L3_HEIGHT, subLED);
        }
        break;

      case PREP_CORAL_L4:
        switch (currentRobotState) {
          case HAS_CORAL:
          case PREP_CORAL_L1:
          case PREP_CORAL_L2:
          case PREP_CORAL_L3:
          case PREP_CORAL_ZERO:
            return new PrepCoralLv(subStateMachine, subElevator, constElevator.CORAL_L4_HEIGHT, subLED);
        }
        break;

      case PREP_CORAL_ZERO:
        switch (currentRobotState) {
          case PREP_CORAL_L1:
          case PREP_CORAL_L2:
          case PREP_CORAL_L3:
          case PREP_CORAL_L4:
          case HAS_CORAL:
            return new PrepCoralZero(subStateMachine, subElevator, subLED);
        }
        break;

      // -- Prep Algae Only --
      case PREP_NET:
        switch (currentRobotState) {
          case HAS_ALGAE:
          case PREP_ALGAE_ZERO:
          case PREP_PROCESSOR:
            return new PrepNet(subStateMachine, subElevator, subAlgaeIntake, subLED);
        }
        break;

      case PREP_PROCESSOR:
        switch (currentRobotState) {
          case HAS_ALGAE:
          case PREP_ALGAE_ZERO:
          case PREP_NET:
            return new PrepProcessor(subStateMachine, subElevator, subAlgaeIntake, subLED);
        }
        break;

      case PREP_ALGAE_ZERO:
        switch (currentRobotState) {
          case HAS_ALGAE:
          case PREP_NET:
          case PREP_PROCESSOR:
            return new PrepAlgaeZero(subStateMachine, subElevator, subAlgaeIntake, subLED);
        }
        break;

      // -- Prep Coral with Algae --
      case PREP_CORAL_L1_WITH_ALGAE:
        break;
      case PREP_CORAL_L2_WITH_ALGAE:
        break;
      case PREP_CORAL_L3_WITH_ALGAE:
        break;
      case PREP_CORAL_L4_WITH_ALGAE:
        break;
      case PREP_CORAL_ZERO_WITH_ALGAE:

        // -- Prep Algae with Coral --
      case PREP_NET_WITH_CORAL:
        break;
      case PREP_PROCESSOR_WITH_CORAL:
        break;
      case PREP_ALGAE_ZERO_WITH_CORAL:

        // -- Scoring --
      case SCORING_CORAL:
        switch (currentRobotState) {
          case PREP_CORAL_L1:
          case PREP_CORAL_L2:
          case PREP_CORAL_L3:
          case PREP_CORAL_L4:
          case PREP_CORAL_ZERO:
            return new ScoringCoral(subCoralOuttake, subStateMachine, subElevator, subLED, conOperator,
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
        break;
      case SCORING_CORAL_WITH_ALGAE:
        break;

      // -- Climbing --
      case CLIMBER_DEPLOYING:
        switch (currentRobotState) {
          case NONE:
          case CLIMBER_RETRACTING:
          case CLIMBER_DEPLOYING:
            return new ClimberDeploying(subStateMachine, subClimber, subElevator, subAlgaeIntake, subLED);
        }
        break;

      case CLIMBER_RETRACTING:
        switch (currentRobotState) {
          case CLIMBER_DEPLOYING:
          case CLIMBER_RETRACTING:
            return new ClimberRetracting(subStateMachine, subClimber, subAlgaeIntake, subLED);
        }
        break;

    }
    return Commands.print("ITS SO OVER D: Invalid State Provided, Blame Eli");
  }

  public static enum DriverState {
    MANUAL,
    REEF_ROTATION_SNAPPING,
    CORAL_STATION_ROTATION_SNAPPING,
    REEF_AUTO_DRIVING,
    CORAL_STATION_AUTO_DRIVING,
    PROCESSOR_ROTATION_SNAPPING,
    PROCESSOR_AUTO_DRIVING,
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
    EJECTING_CORAL,

    // Manip. 2nd Scoring element
    CLEANING_L2_WITH_CORAL,
    CLEANING_L3_WITH_CORAL,
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
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
