// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.lang.Thread.State;
import java.util.List;

import com.frcteam3255.components.swerve.SN_SuperSwerve;
import com.frcteam3255.components.swerve.SN_SwerveModule;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants;
import frc.robot.Constants.*;
import frc.robot.RobotMap.mapDrivetrain;
import frc.robot.subsystems.*;
import frc.robot.subsystems.StateMachine.DriverState;

@Logged
public class Drivetrain extends SN_SuperSwerve {

  private static SN_SwerveModule[] modules = new SN_SwerveModule[] {
      new SN_SwerveModule(0, mapDrivetrain.FRONT_LEFT_DRIVE_CAN, mapDrivetrain.FRONT_LEFT_STEER_CAN,
          mapDrivetrain.FRONT_LEFT_ABSOLUTE_ENCODER_CAN, constDrivetrain.FRONT_LEFT_ABS_ENCODER_OFFSET),
      new SN_SwerveModule(1, mapDrivetrain.FRONT_RIGHT_DRIVE_CAN, mapDrivetrain.FRONT_RIGHT_STEER_CAN,
          mapDrivetrain.FRONT_RIGHT_ABSOLUTE_ENCODER_CAN, constDrivetrain.FRONT_RIGHT_ABS_ENCODER_OFFSET),
      new SN_SwerveModule(2, mapDrivetrain.BACK_LEFT_DRIVE_CAN, mapDrivetrain.BACK_LEFT_STEER_CAN,
          mapDrivetrain.BACK_LEFT_ABSOLUTE_ENCODER_CAN, constDrivetrain.BACK_LEFT_ABS_ENCODER_OFFSET),
      new SN_SwerveModule(3, mapDrivetrain.BACK_RIGHT_DRIVE_CAN, mapDrivetrain.BACK_RIGHT_STEER_CAN,
          mapDrivetrain.BACK_RIGHT_ABSOLUTE_ENCODER_CAN, constDrivetrain.BACK_RIGHT_ABS_ENCODER_OFFSET),
  };

  Pose2d desiredAlignmentPose = Pose2d.kZero;
  SwerveModuleState[] desiredModuleStates;
  SwerveModuleState[] actualModuleStates;

  public Drivetrain() {
    super(
        (RobotContainer.isPracticeBot()) ? constDrivetrain.PRACTICE_BOT.SWERVE_CONSTANTS
            : constDrivetrain.SWERVE_CONSTANTS,
        (RobotContainer.isPracticeBot()) ? constDrivetrain.PRACTICE_BOT.MODULES
            : constDrivetrain.MODULES,
        constDrivetrain.WHEELBASE,
        constDrivetrain.TRACK_WIDTH,
        mapDrivetrain.CAN_BUS_NAME,
        mapDrivetrain.PIGEON_CAN,
        constDrivetrain.MIN_STEER_PERCENT,
        constDrivetrain.DRIVE_MOTOR_INVERT,
        constDrivetrain.STEER_MOTOR_INVERT,
        constDrivetrain.CANCODER_INVERT,
        constDrivetrain.DRIVE_NEUTRAL_MODE,
        constDrivetrain.STEER_NEUTRAL_MODE,
        VecBuilder.fill(
            constDrivetrain.MEASUREMENT_STD_DEVS_POS,
            constDrivetrain.MEASUREMENT_STD_DEVS_POS,
            constDrivetrain.MEASUREMENT_STD_DEV_HEADING),
        VecBuilder.fill(
            constVision.MEGA_TAG2_STD_DEVS_POSITION,
            constVision.MEGA_TAG2_STD_DEVS_POSITION,
            constVision.MEGA_TAG2_STD_DEVS_HEADING),
        constDrivetrain.AUTO.AUTO_DRIVE_PID,
        constDrivetrain.AUTO.AUTO_STEER_PID,
        constDrivetrain.AUTO.ROBOT_CONFIG,
        () -> constField.isRedAlliance(),
        Robot.isSimulation());
  }

  @Override
  public void configure() {
    SN_SwerveModule.driveConfiguration = constDrivetrain.DRIVE_CONFIG;
    SN_SwerveModule.steerConfiguration = constDrivetrain.STEER_CONFIG;
    SN_SwerveModule.cancoderConfiguration = constDrivetrain.CANCODER_CONFIG;
    super.configure();
  }

  public void addEventToAutoMap(String key, Command command) {
    super.autoEventMap.put(key, command);
  }

  /**
   * Returns the rotational velocity calculated with PID control to reach the
   * given rotation. This must be called every loop until you reach the given
   * rotation.
   * 
   * @param desiredYaw The desired yaw to rotate to
   * @return The desired velocity needed to rotate to that position.
   */
  public AngularVelocity getVelocityToRotate(Rotation2d desiredYaw) {
    double yawSetpoint = constDrivetrain.TELEOP_AUTO_ALIGN.TELEOP_AUTO_ALIGN_CONTROLLER.getThetaController()
        .calculate(getRotation().getRadians(), desiredYaw.getRadians());

    // limit the PID output to our maximum rotational speed
    yawSetpoint = MathUtil.clamp(yawSetpoint, -constDrivetrain.TURN_SPEED.in(Units.RadiansPerSecond),
        constDrivetrain.TURN_SPEED.in(Units.RadiansPerSecond));

    return Units.RadiansPerSecond.of(yawSetpoint);
  }

  /**
   * Returns the rotational velocity calculated with PID control to reach the
   * given rotation. This must be called every loop until you reach the given
   * rotation.
   * 
   * @param desiredYaw The desired yaw to rotate to
   * @return The desired velocity needed to rotate to that position.
   */
  public AngularVelocity getVelocityToRotate(Angle desiredYaw) {
    return getVelocityToRotate(Rotation2d.fromDegrees(desiredYaw.in(Units.Degrees)));
  }

  public Angle getRotationMeasure() {
    return Units.Degrees.of(getRotation().getDegrees());
  }

  /**
   * Calculate the ChassisSpeeds needed to align the robot to the desired pose.
   * This must be called every loop until you reach the desired pose.
   * 
   * @param desiredPose The desired pose to align to
   * @return The ChassisSpeeds needed to align the robot to the desired pose
   */
  public ChassisSpeeds getAlignmentSpeeds(Pose2d desiredPose) {
    desiredAlignmentPose = desiredPose;
    // TODO: This might run better if instead of 0, we use
    // constDrivetrain.TELEOP_AUTO_ALIGN.DESIRED_AUTO_ALIGN_SPEED.in(Units.MetersPerSecond);.
    // I dont know why. it might though
    return constDrivetrain.TELEOP_AUTO_ALIGN.TELEOP_AUTO_ALIGN_CONTROLLER.calculate(getPose(), desiredPose, 0,
        desiredPose.getRotation());
  }

  /**
   * Returns the desired reef branch (or branch face) for the robot to auto-align
   * to based on our Pose.
   * 
   * @param leftBranchRequested If we are requesting to align to the left or right
   *                            branch
   * @return The desired reef branch face to align to
   */
  public Pose2d getDesiredReef(boolean leftBranchRequested, StateMachine subStateMachine) {
    Boolean onRed = constField.isRedAlliance();
    boolean poseOnRed = getPose().getX() > 8.775;
    Distance reefDistance = Units.Meters
        .of(getPose().getTranslation()
            .getDistance(constField.getAllFieldPositions(onRed, false).get()[13].getTranslation()));

    if (reefDistance.lte(constDrivetrain.TELEOP_AUTO_ALIGN.MAX_AUTO_DRIVE_REEF_DISTANCE)) {
      // Determine closest reef BRANCH based on our rotation
      List<Pose2d> reefPoses = constField.getReefPositions(poseOnRed).get();
      // Pose2d desiredReef;
      // if (subStateMachine.inAlgaeWithCoralState()) {
      // desiredReef = getClosestPoseByRotation(reefPoses);
      // } else {
      // desiredReef = getClosestPoseByRotation(reefPoseClose);
      // }
      Pose2d desiredReef = getClosestPoseByRotation(reefPoses);
      int closestReefIndex = reefPoses.indexOf(desiredReef);
      // -- The above code will be different later --

      // Invert faces on the back of the reef so they're always relative to the driver
      if (closestReefIndex > 3 && closestReefIndex < 10) {
        leftBranchRequested = !leftBranchRequested;
      }

      // If we were closer to the left branch but selected the right branch (or
      // vice-versa), switch to our desired branch
      if (leftBranchRequested && (closestReefIndex % 2 == 1)) {
        desiredReef = reefPoses.get(closestReefIndex - 1);
      } else if (!leftBranchRequested && (closestReefIndex % 2 == 0)) {
        desiredReef = reefPoses.get(closestReefIndex + 1);
      }
      return desiredReef;
    }
    // Determine the closest reef FACE based on our position (left vs right doesn't
    // matter)
    List<Pose2d> reefPoses = constField.getReefPositions(poseOnRed).get();
    Pose2d desiredReef = getPose().nearest(reefPoses);
    return desiredReef;
  }

  public Pose2d getDesiredAlgae() {
    // Get closest cage
    boolean onRed = getPose().getX() > 8.775;
    List<Pose2d> AlgaePoses = constField.getAlgaePositions(onRed).get();
    Pose2d currentPose = getPose();
    Pose2d desiredAlgae = currentPose.nearest(AlgaePoses);

    return desiredAlgae;
  }

  public Pose2d getDesiredCoralStation(boolean farCoralStationRequested) {
    // Get the closest coral station
    List<Pose2d> coralStationPoses = constField.getCoralStationPositions().get();
    Pose2d currentPose = getPose();
    Pose2d desiredCoralStation = currentPose.nearest(coralStationPoses);
    int closestCoralStationIndex = coralStationPoses.indexOf(desiredCoralStation);

    // If we were closer to the left branch but selected the right branch (or
    // vice-versa), switch to our desired branch
    if (farCoralStationRequested && (closestCoralStationIndex % 2 == 1)) {
      desiredCoralStation = coralStationPoses.get(closestCoralStationIndex - 1);
    } else if (!farCoralStationRequested && (closestCoralStationIndex % 2 == 0)) {
      desiredCoralStation = coralStationPoses.get(closestCoralStationIndex + 1);
    }

    return desiredCoralStation;
  }

  public Pose2d getDesiredProcessor() {
    // Get the closest processor
    List<Pose2d> processorPoses = constField.getProcessorPositions().get();
    Pose2d currentPose = getPose();
    Pose2d desiredProcessor = currentPose.nearest(processorPoses);
    return desiredProcessor;
  }

  public final Double CURRENT_POSE_Y_VALUE = getPose().getY();

  /**
   * Drive the drivetrain with pre-calculated ChassisSpeeds
   *
   * @param chassisSpeeds Desired ChassisSpeeds
   * @param isOpenLoop    Are the modules being set based on open loop or closed
   *                      loop control
   *
   */
  public void drive(ChassisSpeeds chassisSpeeds, boolean isOpenLoop) {
    SwerveModuleState[] desiredModuleStates = swerveKinematics
        .toSwerveModuleStates(ChassisSpeeds.discretize(chassisSpeeds, timeFromLastUpdate));
    setModuleStates(desiredModuleStates, isOpenLoop);
  }

  /**
   * Aligns the drivetrain to a desired rotation.
   * 
   */
  public void rotationalAlign(Pose2d desiredTarget, LinearVelocity xVelocity, LinearVelocity yVelocity,
      boolean isOpenLoop, DriverState rotating, StateMachine subStateMachine) {
    int redAllianceMultiplier = constField.isRedAlliance() ? -1 : 1;
    // Rotational-only auto-align
    drive(
        new Translation2d(xVelocity.times(redAllianceMultiplier).in(Units.MetersPerSecond),
            yVelocity.times(redAllianceMultiplier).in(Units.MetersPerSecond)),
        getVelocityToRotate(desiredTarget.getRotation()).in(Units.RadiansPerSecond), isOpenLoop);
    subStateMachine.setDriverState(rotating);
  }

  public void reefAutoAlign(boolean leftBranchRequested,
      LinearVelocity xVelocity,
      LinearVelocity yVelocity,
      AngularVelocity rVelocity, double elevatorMultiplier, boolean isOpenLoop, Distance maxAutoDriveDistance,
      DriverState driving, DriverState rotating, StateMachine subStateMachine, boolean lockX, boolean lockY) {

    Pose2d desiredReef = getDesiredReef(leftBranchRequested, subStateMachine);
    Boolean onRed = getPose().getX() > 8.775;
    Distance reefDistance = Units.Meters
        .of(getPose().getTranslation()
            .getDistance(constField.getAllFieldPositions(onRed, false).get()[13].getTranslation()));

    autoAlign(reefDistance, desiredReef, xVelocity, yVelocity, rVelocity, elevatorMultiplier, isOpenLoop,
        maxAutoDriveDistance, driving, rotating, subStateMachine, lockX, lockY);
  }

  public void algaeAutoAlign(LinearVelocity xVelocity,
      LinearVelocity yVelocity,
      AngularVelocity rVelocity, double elevatorMultiplier, boolean isOpenLoop, Distance maxAutoDriveDistance,
      DriverState driving, DriverState rotating, StateMachine subStateMachine, boolean lockX, boolean lockY) {

    Pose2d desiredAlgae = getDesiredAlgae();
    Boolean onRed = getPose().getX() > 8.775;
    Distance reefDistance = Units.Meters
        .of(getPose().getTranslation()
            .getDistance(constField.getAllFieldPositions(onRed, false).get()[13].getTranslation()));

    autoAlign(reefDistance, desiredAlgae, xVelocity, yVelocity, rVelocity, elevatorMultiplier, isOpenLoop,
        maxAutoDriveDistance, driving, rotating, subStateMachine, lockX, lockY);
  }

  public void autoPeriodNetAlign(StateMachine subStateMachine) {
    Pose2d desiredNetPose;
    Pose2d currentPose = getPose();
    Pose2d netPose = currentPose.nearest(constField.POSES.NET_POSES);
    desiredNetPose = new Pose2d(netPose.getX(), currentPose.getY(), netPose.getRotation());

    Distance netDistance = Units.Meters
        .of(currentPose.getTranslation().getDistance(desiredNetPose.getTranslation()));

    autoAlign(netDistance, desiredNetPose, MetersPerSecond.of(0), MetersPerSecond.of(0), DegreesPerSecond.of(0),
        1, false, Units.Meters.of(1000),
        DriverState.NET_AUTO_DRIVING, DriverState.NET_ROTATION_SNAPPING, subStateMachine, false, false);
  }

  /**
   * Contains logic for automatically aligning & automatically driving to a pose.
   * May align only rotationally or automatically drive to the pose.
   */
  public void autoAlign(Distance distanceFromTarget, Pose2d desiredTarget,
      LinearVelocity xVelocity, LinearVelocity yVelocity, AngularVelocity rVelocity, double elevatorMultiplier,
      boolean isOpenLoop, Distance maxAutoDriveDistance, DriverState driving, DriverState rotating,
      StateMachine subStateMachine, boolean lockX, boolean lockY) {
    desiredAlignmentPose = desiredTarget;
    int redAllianceMultiplier = constField.isRedAlliance() ? -1 : 1;
    double manualXVelocity = xVelocity.times(redAllianceMultiplier).in(Units.MetersPerSecond);
    double manualYVelocity = yVelocity.times(redAllianceMultiplier).in(Units.MetersPerSecond);
    if (distanceFromTarget.gte(maxAutoDriveDistance)) {
      // Rotational-only auto-align
      rotationalAlign(desiredTarget, xVelocity, yVelocity, isOpenLoop, rotating, subStateMachine);
    } else {
      // Full auto-align
      ChassisSpeeds automatedDTVelocity = getAlignmentSpeeds(desiredTarget);
      subStateMachine.setDriverState(driving);

      // Speed limit based on elevator height
      LinearVelocity linearSpeedLimit = constDrivetrain.OBSERVED_DRIVE_SPEED.times(elevatorMultiplier);
      AngularVelocity angularSpeedLimit = constDrivetrain.TURN_SPEED.times(elevatorMultiplier);

      if (!RobotState.isAutonomous()) {
        if ((automatedDTVelocity.vxMetersPerSecond > linearSpeedLimit.in(Units.MetersPerSecond))
            || (automatedDTVelocity.vyMetersPerSecond > linearSpeedLimit.in(Units.MetersPerSecond))
            || (automatedDTVelocity.omegaRadiansPerSecond > angularSpeedLimit.in(Units.RadiansPerSecond))) {

          // Automated calculated velocity
          automatedDTVelocity.vxMetersPerSecond = MathUtil.clamp(automatedDTVelocity.vxMetersPerSecond, 0,
              linearSpeedLimit.in(MetersPerSecond));
          automatedDTVelocity.vyMetersPerSecond = MathUtil.clamp(automatedDTVelocity.vyMetersPerSecond, 0,
              linearSpeedLimit.in(MetersPerSecond));
          automatedDTVelocity.omegaRadiansPerSecond = MathUtil.clamp(automatedDTVelocity.omegaRadiansPerSecond, 0,
              angularSpeedLimit.in(RadiansPerSecond));
        }
      }
      if (lockX) {
        automatedDTVelocity.vxMetersPerSecond = manualXVelocity;
      }
      if (lockY) {
        automatedDTVelocity.vyMetersPerSecond = manualYVelocity;
      }
      drive(automatedDTVelocity, isOpenLoop);
    }
  }

  public boolean isAtRotation(Rotation2d desiredRotation) {
    return (getRotation().getMeasure()
        .compareTo(desiredRotation.getMeasure().minus(constDrivetrain.TELEOP_AUTO_ALIGN.AT_ROTATION_TOLERANCE)) > 0) &&
        getRotation().getMeasure()
            .compareTo(desiredRotation.getMeasure().plus(constDrivetrain.TELEOP_AUTO_ALIGN.AT_ROTATION_TOLERANCE)) < 0;
  }

  public boolean isAtRotation(Rotation2d desiredRotation, Angle tolerance) {
    return (getRotation().getMeasure()
        .compareTo(desiredRotation.getMeasure().minus(tolerance)) > 0) &&
        getRotation().getMeasure()
            .compareTo(desiredRotation.getMeasure().plus(tolerance)) < 0;
  }

  public boolean isAtPosition(Pose2d desiredPose2d) {
    return Units.Meters
        .of(getPose().getTranslation().getDistance(desiredPose2d.getTranslation()))
        .lte(constDrivetrain.TELEOP_AUTO_ALIGN.AT_POINT_TOLERANCE);
  }

  public Boolean isAlignedCoral() {
    return (desiredAlignmentPose.getTranslation().getDistance(
        getPose().getTranslation()) <= constDrivetrain.TELEOP_AUTO_ALIGN.AUTO_ALIGNMENT_CORAL_TOLERANCE
            .in(Units.Meters))
        && isAtRotation(desiredAlignmentPose.getRotation());
  }

  public Boolean isAlignedAlgae() {
    return (desiredAlignmentPose.getTranslation().getDistance(
        getPose().getTranslation()) <= constDrivetrain.TELEOP_AUTO_ALIGN.AUTO_ALIGNMENT_ALGAE_TOLERANCE
            .in(Units.Meters))
        && isAtRotation(desiredAlignmentPose.getRotation());
  }

  public Boolean isAlignedNet() {
    return (desiredAlignmentPose.getTranslation().getDistance(
        getPose().getTranslation()) <= constDrivetrain.TELEOP_AUTO_ALIGN.AUTO_ALIGN_NET_TOLERANCE
            .in(Units.Meters))
        && isAtRotation(desiredAlignmentPose.getRotation(), constDrivetrain.TELEOP_AUTO_ALIGN.ROTATED_NET_TOLERANCE);
  }

  public boolean atPose(Pose2d desiredPose) {
    return isAtRotation(desiredPose.getRotation()) && isAtPosition(desiredPose);
  }

  /**
   * Calculate which pose from an array has the closest rotation to the robot's
   * current pose. If multiple poses have the same rotation, the last one in the
   * list will be returned.
   * 
   * @param poses The list of poses to check
   * @return The last pose in the list with the closest rotation
   */
  public Pose2d getClosestPoseByRotation(List<Pose2d> poses) {
    Pose2d closestPose = poses.get(0);
    double smallestDifference = Math.abs(getRotation().minus(closestPose.getRotation()).getRadians());
    for (Pose2d pose : poses) {
      double difference = Math.abs(getRotation().minus(pose.getRotation()).getRadians());
      if (difference < smallestDifference) {
        smallestDifference = difference;
        closestPose = pose;
      }
    }
    return closestPose;
  }

  @Override
  public void periodic() {
    super.periodic();

    for (SN_SwerveModule mod : modules) {
      SmartDashboard.putNumber("Drivetrain/Module " + mod.moduleNumber + "/Desired Speed (FPS)",
          Units.Feet.convertFrom(Math.abs(getDesiredModuleStates()[mod.moduleNumber].speedMetersPerSecond),
              Units.Meters));
      SmartDashboard.putNumber("Drivetrain/Module " + mod.moduleNumber + "/Actual Speed (FPS)",
          Units.Feet.convertFrom(Math.abs(getActualModuleStates()[mod.moduleNumber].speedMetersPerSecond),
              Units.Meters));

      SmartDashboard.putNumber("Drivetrain/Module " + mod.moduleNumber + "/Desired Angle (Degrees)",
          Math.abs(getDesiredModuleStates()[mod.moduleNumber].angle.getDegrees()));
      SmartDashboard.putNumber("Drivetrain/Module " + mod.moduleNumber + "/Actual Angle (Degrees)",
          Math.abs(getActualModuleStates()[mod.moduleNumber].angle.getDegrees()));

      SmartDashboard.putNumber("Drivetrain/Module " + mod.moduleNumber + "/Offset Absolute Encoder Angle (Rotations)",
          mod.getAbsoluteEncoder());
      SmartDashboard.putNumber("Drivetrain/Module " + mod.moduleNumber + "/Absolute Encoder Raw Value (Rotations)",
          mod.getRawAbsoluteEncoder());
    }

    field.setRobotPose(getPose());
    desiredModuleStates = getDesiredModuleStates();
    actualModuleStates = getActualModuleStates();

    SmartDashboard.putData(field);
    SmartDashboard.putNumber("Drivetrain/Rotation", getRotationMeasure().in(Units.Degrees));
  }
}
