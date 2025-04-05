// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.frcteam3255.utils.LimelightHelpers;
import com.frcteam3255.utils.LimelightHelpers.PoseEstimate;
import java.util.Optional;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.AngularVelocity;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constVision;

@Logged
public class Vision extends SubsystemBase {
  PoseEstimate lastEstimateRight = new PoseEstimate();
  PoseEstimate lastEstimateLeft = new PoseEstimate();
  PoseEstimate lastEstimateBack = new PoseEstimate();

  // Not logged, as they turn to false immediately after being read
  @NotLogged
  boolean newRightEstimate = false;
  @NotLogged
  boolean newLeftEstimate = false;
  @NotLogged
  boolean newBackEstimate = false;

  Pose2d rightPose = new Pose2d();
  Pose2d leftPose = new Pose2d();
  Pose2d backPose = new Pose2d();

  private boolean useMegaTag2 = false;

  public Vision() {
  }

  public PoseEstimate[] getLastPoseEstimates() {
    return new PoseEstimate[] { lastEstimateRight, lastEstimateLeft, lastEstimateBack };
  }

  public void setMegaTag2(boolean useMegaTag2) {
    this.useMegaTag2 = useMegaTag2;
  }

  /**
   * Determines if a given pose estimate should be rejected.
   * 
   * 
   * @param poseEstimate The pose estimate to check
   * @param gyroRate     The current rate of rotation observed by our gyro.
   * 
   * @return True if the estimate should be rejected
   */

  public boolean rejectUpdate(PoseEstimate poseEstimate, AngularVelocity gyroRate) {
    // Angular velocity is too high to have accurate vision
    if (gyroRate.compareTo(constVision.MAX_ANGULAR_VELOCITY) > 0) {
      return true;
    }

    // No tags :<
    if (poseEstimate.tagCount == 0) {
      return true;
    }

    // 1 Tag with a large area
    if (poseEstimate.tagCount == 1 && poseEstimate.avgTagArea > constVision.AREA_THRESHOLD) {
      return false;
      // 2 tags or more
    } else if (poseEstimate.tagCount > 1) {
      return false;
    }

    return true;
  }

  /**
   * Updates the current pose estimates for the left and right of the robot using
   * data from Limelight cameras.
   *
   * @param gyroRate The current angular velocity of the robot, used to validate
   *                 the pose estimates.
   *
   *                 This method retrieves pose estimates from two Limelight
   *                 cameras (left and right) and updates the
   *                 corresponding pose estimates if they are valid. The method
   *                 supports two modes of operation:
   *                 one using MegaTag2 and one without. The appropriate pose
   *                 estimate retrieval method is chosen
   *                 based on the value of the `useMegaTag2` flag.
   *
   *                 If the retrieved pose estimates are valid and not rejected
   *                 based on the current angular velocity,
   *                 the method updates the last known estimates and sets flags
   *                 indicating new estimates are available.
   */
  public void setCurrentEstimates(AngularVelocity gyroRate) {
    PoseEstimate currentEstimateRight = new PoseEstimate();
    PoseEstimate currentEstimateLeft = new PoseEstimate();
    PoseEstimate currentEstimateBack = new PoseEstimate();

    if (useMegaTag2) {
      currentEstimateRight = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(constVision.LIMELIGHT_NAMES[0]);
      currentEstimateLeft = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(constVision.LIMELIGHT_NAMES[1]);
      currentEstimateBack = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(constVision.LIMELIGHT_NAMES[2]);
    } else {
      currentEstimateRight = LimelightHelpers.getBotPoseEstimate_wpiBlue(constVision.LIMELIGHT_NAMES[0]);
      currentEstimateLeft = LimelightHelpers.getBotPoseEstimate_wpiBlue(constVision.LIMELIGHT_NAMES[1]);
      currentEstimateBack = LimelightHelpers.getBotPoseEstimate_wpiBlue(constVision.LIMELIGHT_NAMES[2]);
    }

    if (currentEstimateRight != null && !rejectUpdate(currentEstimateRight, gyroRate)) {
      lastEstimateRight = currentEstimateRight;
      rightPose = currentEstimateRight.pose;
      newRightEstimate = true;
    }
    if (currentEstimateLeft != null && !rejectUpdate(currentEstimateLeft, gyroRate)) {
      lastEstimateLeft = currentEstimateLeft;
      leftPose = currentEstimateLeft.pose;
      newLeftEstimate = true;
    }
    if (currentEstimateBack != null && !rejectUpdate(currentEstimateBack, gyroRate)) {
      lastEstimateBack = currentEstimateBack;
      backPose = currentEstimateBack.pose;
      newBackEstimate = true;
    }
  }

  public Optional<PoseEstimate> determinePoseEstimate(AngularVelocity gyroRate) {
    setCurrentEstimates(gyroRate);

    // No valid pose estimates :(
    if (!newRightEstimate && !newLeftEstimate && !newBackEstimate) {
      return Optional.empty();

    } else if (newRightEstimate && !newLeftEstimate && !newBackEstimate) {
      // One valid pose estimate (right)
      newRightEstimate = false;
      return Optional.of(lastEstimateRight);

    } else if (!newRightEstimate && newLeftEstimate && !newBackEstimate) {
      // One valid pose estimate (left)
      newLeftEstimate = false;
      return Optional.of(lastEstimateLeft);

    } else if (!newRightEstimate && !newLeftEstimate && newBackEstimate) {
      // One valid pose estimate (back)
      newLeftEstimate = false;
      return Optional.of(lastEstimateBack);

    } else {
      // More than one valid pose estimate, use the closest one
      newRightEstimate = false;
      newLeftEstimate = false;
      newBackEstimate = false;
      if (lastEstimateRight.avgTagDist < lastEstimateLeft.avgTagDist
          && lastEstimateRight.avgTagDist < lastEstimateBack.avgTagDist) {
        return Optional.of(lastEstimateRight);
      } else if (lastEstimateLeft.avgTagDist < lastEstimateRight.avgTagDist
          && lastEstimateLeft.avgTagDist < lastEstimateBack.avgTagDist) {
        return Optional.of(lastEstimateLeft);
      } else {
        return Optional.of(lastEstimateBack);
      }
    }
  }

  @Override
  public void periodic() {
  }
}
