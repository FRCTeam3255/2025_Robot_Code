// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constVision;

@Logged
public class Vision extends SubsystemBase {
  private final PhotonCamera leftCamera;
  private final PhotonCamera rightCamera;
  private final PhotonCamera backCamera;
  private final PhotonPoseEstimator photonEstimatorLeft;
  private final PhotonPoseEstimator photonEstimatorRight;
  private final PhotonPoseEstimator photonEstimatorBack;
  @NotLogged
  private Matrix<N3, N1> curStdDevs;
  private final EstimateConsumer estConsumer;

  public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  public Vision(EstimateConsumer estConsumer) {
    this.estConsumer = estConsumer;
    leftCamera = new PhotonCamera("Front-Left-Cam");
    rightCamera = new PhotonCamera("Front-Right-Cam");
    backCamera = new PhotonCamera("Back-Cam");
    photonEstimatorLeft = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        constVision.kRobotToLeftCam);
    photonEstimatorLeft.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    photonEstimatorRight = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        constVision.kRobotToRightCam);
    photonEstimatorRight.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    photonEstimatorBack = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        constVision.kRobotToBackCam);
    photonEstimatorBack.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  private void updateEstimationStdDevs(
      Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
    if (estimatedPose.isEmpty()) {
      // No pose input. Default to single-tag std devs
      curStdDevs = constVision.kSingleTagStdDevs;

    } else {
      // Pose present. Start running Heuristic
      var estStdDevs = constVision.kSingleTagStdDevs;
      int numTags = 0;
      double avgDist = 0;

      // Precalculation - see how many tags we found, and calculate an
      // average-distance metric
      for (var tgt : targets) {
        var tagPose = photonEstimatorLeft.getFieldTags().getTagPose(tgt.getFiducialId());
        if (tagPose.isEmpty())
          continue;
        numTags++;
        avgDist += tagPose
            .get()
            .toPose2d()
            .getTranslation()
            .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
      }

      for (var tgt : targets) {
        var tagPose = photonEstimatorRight.getFieldTags().getTagPose(tgt.getFiducialId());
        if (tagPose.isEmpty())
          continue;
        numTags++;
        avgDist += tagPose
            .get()
            .toPose2d()
            .getTranslation()
            .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
      }

      for (var tgt : targets) {
        var tagPose = photonEstimatorBack.getFieldTags().getTagPose(tgt.getFiducialId());
        if (tagPose.isEmpty())
          continue;
        numTags++;
        avgDist += tagPose
            .get()
            .toPose2d()
            .getTranslation()
            .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
      }

      if (numTags == 0) {
        // No tags visible. Default to single-tag std devs
        curStdDevs = constVision.kSingleTagStdDevs;
      } else {
        // One or more tags visible, run the full heuristic.
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1)
          estStdDevs = constVision.kMultiTagStdDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
          estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else
          estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
        curStdDevs = estStdDevs;
      }
    }
  }

  @Override
  public void periodic() {
    Optional<EstimatedRobotPose> visionEstLeft = Optional.empty();
    for (var change : leftCamera.getAllUnreadResults()) {
      visionEstLeft = photonEstimatorLeft.update(change);
      updateEstimationStdDevs(visionEstLeft, change.getTargets());
      visionEstLeft.ifPresent(
          est -> {
            // Change our trust in the measurement based on the tags we can see
            var estStdDevs = getEstimationStdDevs();

            estConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds);
          });
    }

    Optional<EstimatedRobotPose> visionEstRight = Optional.empty();
    for (var change : rightCamera.getAllUnreadResults()) {
      visionEstRight = photonEstimatorRight.update(change);
      updateEstimationStdDevs(visionEstRight, change.getTargets());
      visionEstRight.ifPresent(
          est -> {
            // Change our trust in the measurement based on the tags we can see
            var estStdDevs = getEstimationStdDevs();

            estConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds);
          });
    }

    Optional<EstimatedRobotPose> visionEstBack = Optional.empty();
    for (var change : backCamera.getAllUnreadResults()) {
      visionEstBack = photonEstimatorBack.update(change);
      updateEstimationStdDevs(visionEstBack, change.getTargets());
      visionEstBack.ifPresent(
          est -> {
            // Change our trust in the measurement based on the tags we can see
            var estStdDevs = getEstimationStdDevs();

            estConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds);
          });
    }
  }

  /**
   * Returns the latest standard deviations of the estimated pose from {@link
   * #getEstimatedGlobalPose()}, for use with {@link
   * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
   * SwerveDrivePoseEstimator}. This should
   * only be used when there are targets visible.
   */
  @NotLogged
  public Matrix<N3, N1> getEstimationStdDevs() {
    return curStdDevs;
  }

  @FunctionalInterface
  public static interface EstimateConsumer {
    public void accept(Pose2d pose, double timestamp);
  }
}
