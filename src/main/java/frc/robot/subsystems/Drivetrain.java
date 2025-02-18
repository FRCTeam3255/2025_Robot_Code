// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.List;

import com.frcteam3255.components.swerve.SN_SuperSwerve;
import com.frcteam3255.components.swerve.SN_SwerveModule;
import com.frcteam3255.utils.SN_Math;

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.constDrivetrain;
import frc.robot.Constants.constField;
import frc.robot.Constants.constVision;
import frc.robot.RobotMap.mapDrivetrain;
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

  public Boolean isAligned() {
    return desiredAlignmentPose.getTranslation().getDistance(
        getPose().getTranslation()) <= constDrivetrain.TELEOP_AUTO_ALIGN.AUTO_ALIGNMENT_TOLERANCE.in(Units.Meters);
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
   * Returns the closest reef branch to the robot.
   * 
   * @param leftBranchRequested If we are requesting to align to the left or right
   *                            branch
   * @return The desired reef branch face to align to
   */
  public Pose2d getDesiredReef(boolean leftBranchRequested) {
    // Get the closest reef branch face using either branch on the face
    List<Pose2d> reefPoses = constField.getReefPositions().get();
    Pose2d currentPose = getPose();
    Pose2d desiredReef = currentPose.nearest(reefPoses);
    int closestReefIndex = reefPoses.indexOf(desiredReef);

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

  public Pose2d getDesiredProcessor() {
    // Get the closest processor
    List<Pose2d> processorPoses = constField.getProcessorPositions().get();
    Pose2d currentPose = getPose();
    Pose2d desiredProcessor = currentPose.nearest(processorPoses);

    return desiredProcessor;
  }

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
   * Contains logic for automatically aligning & automatically driving to the
   * reef.
   * May align only rotationally, automatically drive to a branch, or be
   * overridden by the driver
   */

  public void autoAlign(Distance distanceFromTarget, Pose2d desiredTarget,
      LinearVelocity xVelocity,
      LinearVelocity yVelocity,
      AngularVelocity rVelocity, double elevatorMultiplier, boolean isOpenLoop, Distance maxAutoDriveDistance,
      DriverState driving, DriverState rotating, StateMachine subStateMachine) {
    desiredAlignmentPose = desiredTarget;

    if (distanceFromTarget.gte(maxAutoDriveDistance)) {
      // Rotational-only auto-align
      drive(new Translation2d(xVelocity.in(Units.MetersPerSecond), yVelocity.in(Units.MetersPerSecond)),
          getVelocityToRotate(desiredTarget.getRotation()).in(Units.RadiansPerSecond), isOpenLoop);
      subStateMachine.setDriverState(rotating);
    } else {
      // Full auto-align
      ChassisSpeeds desiredChassisSpeeds = getAlignmentSpeeds(desiredTarget);
      subStateMachine.setDriverState(driving);

      // Speed limit based on elevator height
      LinearVelocity linearSpeedLimit = constDrivetrain.OBSERVED_DRIVE_SPEED.times(elevatorMultiplier);
      AngularVelocity angularSpeedLimit = constDrivetrain.TURN_SPEED.times(elevatorMultiplier);

      if ((desiredChassisSpeeds.vxMetersPerSecond > linearSpeedLimit.in(Units.MetersPerSecond))
          || (desiredChassisSpeeds.vyMetersPerSecond > linearSpeedLimit.in(Units.MetersPerSecond))
          || (desiredChassisSpeeds.omegaRadiansPerSecond > angularSpeedLimit.in(Units.RadiansPerSecond))) {

        desiredChassisSpeeds.vxMetersPerSecond = MathUtil.clamp(desiredChassisSpeeds.vxMetersPerSecond, 0,
            linearSpeedLimit.in(MetersPerSecond));
        desiredChassisSpeeds.vyMetersPerSecond = MathUtil.clamp(desiredChassisSpeeds.vyMetersPerSecond, 0,
            linearSpeedLimit.in(MetersPerSecond));
        desiredChassisSpeeds.omegaRadiansPerSecond = MathUtil.clamp(desiredChassisSpeeds.omegaRadiansPerSecond, 0,
            angularSpeedLimit.in(RadiansPerSecond));
      }

      drive(desiredChassisSpeeds, isOpenLoop);
    }
  }

  public boolean isAtRotation(Rotation2d desiredRotation) {
    return (getRotation().getMeasure()
        .compareTo(desiredRotation.getMeasure().minus(constDrivetrain.TELEOP_AUTO_ALIGN.AT_ROTATION_TOLERANCE)) > 0) &&
        getRotation().getMeasure()
            .compareTo(desiredRotation.getMeasure().plus(constDrivetrain.TELEOP_AUTO_ALIGN.AT_ROTATION_TOLERANCE)) < 0;
  }

  public boolean isAtPosition(Pose2d desiredPose2d) {
    return Units.Meters
        .of(getPose().getTranslation().getDistance(desiredPose2d.getTranslation()))
        .lte(constDrivetrain.TELEOP_AUTO_ALIGN.AT_POINT_TOLERANCE);
  }

  public boolean atPose(Pose2d desiredPose) {
    return isAtRotation(desiredPose.getRotation()) && isAtPosition(desiredPose);
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
