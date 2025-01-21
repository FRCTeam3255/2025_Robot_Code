// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.constElevator;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.states.*;

@Logged
public class StateMachine extends SubsystemBase {
  public static RobotState currentRobotState;
  public static TargetState currentTargetState;
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
  StateMachine subStateMachine = this;

  /** Creates a new StateMachine. */
  public StateMachine(AlgaeIntake subAlgaeIntake, Climber subClimber, CoralOuttake subCoralOuttake,
      Drivetrain subDrivetrain, Elevator subElevator, Hopper subHopper) {
    currentRobotState = RobotState.NONE;
    currentTargetState = TargetState.NONE;

    this.subAlgaeIntake = subAlgaeIntake;
    this.subClimber = subClimber;
    this.subCoralOuttake = subCoralOuttake;
    this.subDrivetrain = subDrivetrain;
    this.subElevator = subElevator;
    this.subHopper = subHopper;
  }

  public void setRobotState(RobotState robotState) {
    currentRobotState = robotState;
  }

  public void setTargetState(TargetState targetState) {
    currentTargetState = targetState;
  }

  public RobotState getRobotState() {
    return currentRobotState;
  }

  public TargetState getTargetState() {
    return currentTargetState;
  }

  public Command tryState(RobotState desiredState) {
    switch (desiredState) {
      case NONE:
        switch (currentRobotState) {
          case INTAKING_CORAL_HOPPER:
          case INTAKING_ALGAE_GROUND:
          case EJECTING_CORAL:
          case EJECTING_ALGAE:
          case CLEANING_L2:
          case CLEANING_L3:
          case SCORING_CORAL:
          case SCORING_ALGAE:
          case CLIMBING_DEEP:
            return new None(subStateMachine, subCoralOuttake, subHopper, subAlgaeIntake, subClimber, subElevator);
        }
        break;

      // ------------ Coral -------------
      case INTAKING_CORAL_HOPPER:
        switch (currentRobotState) {
          case NONE:
            return new IntakeCoralHopper(subStateMachine, subHopper, subCoralOuttake);
        }
        break;

      case HAS_CORAL:
        switch (currentRobotState) {
          case INTAKING_CORAL_HOPPER:
            return new HasCoral(subStateMachine, subCoralOuttake);
        }
        break;

      case PREP_CORAL_L1:
        switch (currentRobotState) {
          case HAS_CORAL:
          case PREP_CORAL_L2:
          case PREP_CORAL_L3:
          case PREP_CORAL_L4:
            return new PrepCoralLv(subStateMachine, subElevator, constElevator.CORAL_L1_HEIGHT);
        }
        break;

      case PREP_CORAL_L2:
        switch (currentRobotState) {
          case HAS_CORAL:
          case PREP_CORAL_L1:
          case PREP_CORAL_L3:
          case PREP_CORAL_L4:
            return new PrepCoralLv(subStateMachine, subElevator, constElevator.CORAL_L2_HEIGHT);
        }
        break;

      case PREP_CORAL_L3:
        switch (currentRobotState) {
          case HAS_CORAL:
          case PREP_CORAL_L1:
          case PREP_CORAL_L2:
          case PREP_CORAL_L4:
            return new PrepCoralLv(subStateMachine, subElevator, constElevator.CORAL_L3_HEIGHT);
        }
        break;

      case PREP_CORAL_L4:
        switch (currentRobotState) {
          case HAS_CORAL:
          case PREP_CORAL_L1:
          case PREP_CORAL_L2:
          case PREP_CORAL_L3:
            return new PrepCoralLv(subStateMachine, subElevator, constElevator.CORAL_L4_HEIGHT);
        }
        break;

      case EJECTING_CORAL:
        switch (currentRobotState) {
          case HAS_CORAL:
          case INTAKING_CORAL_HOPPER:
            return new EjectCoral(subStateMachine, subCoralOuttake);
        }
        break;

      case PREP_CORAL_ZERO:
        switch (currentRobotState) {
          case HAS_CORAL:
            return new PrepCoralZero(subStateMachine, subElevator);
        }
        break;

      case SCORING_CORAL:
        switch (currentRobotState) {
          case PREP_CORAL_L1:
          case PREP_CORAL_L2:
          case PREP_CORAL_L3:
          case PREP_CORAL_L4:
          case PREP_CORAL_ZERO:
            return new PlaceCoral(subStateMachine, subCoralOuttake);
        }
        break;

      // ---------- Algae ------------
      case INTAKING_ALGAE_GROUND:
        switch (currentRobotState) {
          case NONE:
            return new IntakingAlgaeGround(subStateMachine, subElevator, subAlgaeIntake);
        }
        break;

      case CLEANING_L2:
        switch (currentRobotState) {
          case NONE:
            return new CleaningL2Reef(subStateMachine, subElevator, subAlgaeIntake);
        }
        break;

      case CLEANING_L3:
        switch (currentRobotState) {
          case NONE:
            return new CleaningL3Reef(subStateMachine, subElevator, subAlgaeIntake);
        }
        break;

      case HAS_ALGAE:
        switch (currentRobotState) {
          case INTAKING_ALGAE_GROUND:
          case CLEANING_L2:
          case CLEANING_L3:
            return new HasAlgae(subStateMachine, subAlgaeIntake);
        }
        break;

      case PREP_NET:
        switch (currentRobotState) {
          case HAS_ALGAE:
          case PREP_PROCESSOR:
            return new PrepNet(subStateMachine, subElevator);
        }
        break;

      case PREP_PROCESSOR:
        switch (currentRobotState) {
          case HAS_ALGAE:
          case PREP_NET:
            return new PrepProcessor(subStateMachine, subElevator);
        }
        break;

      case EJECTING_ALGAE:
        switch (currentRobotState) {
          case INTAKING_ALGAE_GROUND:
          case HAS_ALGAE:
          case CLEANING_L2:
          case CLEANING_L3:
            return new EjectCoral(subStateMachine, subCoralOuttake);
        }
        break;

      case SCORING_ALGAE:
        switch (currentRobotState) {
          case PREP_NET:
          case PREP_PROCESSOR:
          case PREP_ALGAE_ZERO:
            return new ScoringAlgae(subStateMachine, subAlgaeIntake);
        }
        break;

      case PREP_ALGAE_ZERO:
        switch (currentRobotState) {
          case HAS_ALGAE:
            return new PrepCoralZero(subStateMachine, subElevator);
        }
        break;

      case CLIMBING_DEEP:
        switch (currentRobotState) {
          case NONE:
            return new Climb(subStateMachine, subClimber);
        }
        break;

    }
    return Commands.print("ITS SO OVER D: Invalid State Provided, Blame Eli");
  }

  public static enum RobotState {
    NONE,
    INTAKING_CORAL_HOPPER,
    HAS_CORAL,
    PREP_CORAL_L1,
    PREP_CORAL_L2,
    PREP_CORAL_L3,
    PREP_CORAL_L4,
    EJECTING_CORAL,
    SCORING_CORAL,
    PREP_CORAL_ZERO,

    INTAKING_ALGAE_GROUND,
    CLEANING_L2,
    CLEANING_L3,
    HAS_ALGAE,
    PREP_NET,
    PREP_PROCESSOR,
    EJECTING_ALGAE,
    SCORING_ALGAE,
    PREP_ALGAE_ZERO,

    CLIMBING_DEEP
  }

  public static enum TargetState {
    NONE,
    PREP_CORAL_L1,
    PREP_CORAL_L2,
    PREP_CORAL_L3,
    PREP_CORAL_L4,
    PREP_NET,
    PREP_PROCESSOR,
    PREP_ALGAE_0,
    PREP_CORAL_0,
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
