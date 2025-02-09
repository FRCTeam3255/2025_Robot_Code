// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.frcteam3255.utils.LimelightHelpers;
import com.frcteam3255.utils.LimelightHelpers.PoseEstimate;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.measure.AngularVelocity;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constVision;

@Logged
public class Vision extends SubsystemBase {
  PoseEstimate lastEstimateFront = new PoseEstimate();
  PoseEstimate lastEstimateBack = new PoseEstimate();

  Pose2d frontPose = new Pose2d();
  Pose2d backPose = new Pose2d();

  private boolean useMegaTag2 = false;

  public Vision() {
  }

  public PoseEstimate getFrontPoseEstimate() {
    return lastEstimateFront;
  }

  public PoseEstimate getBackPoseEstimate() {
    return lastEstimateBack;
  }

  public PoseEstimate[] getPoseEstimates() {
    return new PoseEstimate[] { getFrontPoseEstimate(), getBackPoseEstimate() };
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

  @Override
  public void periodic() {
    PoseEstimate currentFrontEstimate;
    PoseEstimate currentBackEstimate;

    if (useMegaTag2) {
      currentFrontEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(constVision.LIMELIGHT_NAMES[0]);
      currentBackEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(constVision.LIMELIGHT_NAMES[1]);
    } else {
      currentFrontEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(constVision.LIMELIGHT_NAMES[0]);
      currentBackEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(constVision.LIMELIGHT_NAMES[1]);
    }

    if (currentFrontEstimate != null) {
      lastEstimateFront = currentFrontEstimate;
      frontPose = currentFrontEstimate.pose;
    }
    if (currentBackEstimate != null) {
      lastEstimateBack = currentBackEstimate;
      backPose = currentBackEstimate.pose;
    }
  }
}
