// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import com.frcteam3255.components.swerve.SN_SuperSwerve;
import com.frcteam3255.components.swerve.SN_SwerveModule;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.constDrivetrain;
import frc.robot.Constants.constField;
import frc.robot.Constants.constVision;
import frc.robot.RobotMap.mapDrivetrain;

@Logged
public class Drivetrain extends SN_SuperSwerve {

  StructPublisher<Pose2d> robotPosePublisher = NetworkTableInstance.getDefault()
      .getStructTopic("/SmartDashboard/Drivetrain/Robot Pose", Pose2d.struct).publish();
  StructPublisher<Pose2d> desiredAlignmentPosePublisher = NetworkTableInstance.getDefault()
      .getStructTopic("/SmartDashboard/Drivetrain/Desired Alignment Pose", Pose2d.struct).publish();
  StructArrayPublisher<SwerveModuleState> desiredStatesPublisher = NetworkTableInstance.getDefault()
      .getStructArrayTopic("/SmartDashboard/Drivetrain/Desired States", SwerveModuleState.struct).publish();
  StructArrayPublisher<SwerveModuleState> actualStatesPublisher = NetworkTableInstance.getDefault()
      .getStructArrayTopic("/SmartDashboard/Drivetrain/Actual States", SwerveModuleState.struct).publish();

  Pose2d desiredAlignmentPose = new Pose2d(0, 0, new Rotation2d(0));

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
            constVision.STD_DEVS_POS,
            constVision.STD_DEVS_POS,
            constVision.STD_DEVS_HEADING),
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
    robotPosePublisher.set(getPose());
    desiredAlignmentPosePublisher.set(desiredAlignmentPose);
    desiredStatesPublisher.set(getDesiredModuleStates());
    actualStatesPublisher.set(getActualModuleStates());

    SmartDashboard.putData(field);
    SmartDashboard.putNumber("Drivetrain/Rotation", getRotationMeasure().in(Units.Degrees));
  }
}
