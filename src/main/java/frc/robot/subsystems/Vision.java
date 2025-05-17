// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constVision;
import frc.robot.Robot;

@Logged
public class Vision extends SubsystemBase {
  private final PhotonCamera leftCamera;
  private final PhotonCamera rightCamera;
  private final PhotonPoseEstimator photonEstimator;
  private Matrix<N3, N1> curStdDevs;
  private final EstimateConsumer estConsumer;

  private PhotonCameraSim leftCameraSim;
  private PhotonCameraSim rightCameraSim;
  private VisionSystemSim visionSim;

  public Vision(EstimateConsumer estConsumer) {
    this.estConsumer = estConsumer;
    leftCamera = new PhotonCamera("OV-Face-Up");    
    rightCamera = new PhotonCamera("AR-Face-Up");
    photonEstimator =
    new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, kRobotToCam);
photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  public static final Transform3d kRobotToLeftCam = new Transform3d(
      new Translation3d(constVision.PHOTONVISION_CAM_LEFT.PV_FORWARD, constVision.PHOTONVISION_CAM_LEFT.PV_RIGHT,
          constVision.PHOTONVISION_CAM_LEFT.PV_UP),
      new Rotation3d(constVision.PHOTONVISION_CAM_LEFT.PV_ROLL, constVision.PHOTONVISION_CAM_LEFT.PV_PITCH,
          constVision.PHOTONVISION_CAM_LEFT.PV_YAW));

  public static final Transform3d kRobotToRightCam = new Transform3d(
      new Translation3d(constVision.PHOTONVISION_CAM_RIGHT.PV_FORWARD, constVision.PHOTONVISION_CAM_RIGHT.PV_RIGHT,
          constVision.PHOTONVISION_CAM_RIGHT.PV_UP),
      new Rotation3d(constVision.PHOTONVISION_CAM_RIGHT.PV_ROLL, constVision.PHOTONVISION_CAM_RIGHT.PV_PITCH,
          constVision.PHOTONVISION_CAM_RIGHT.PV_YAW));

  PhotonPoseEstimator photonEstimatorLeft = new PhotonPoseEstimator(
      kTagLayout,
      PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
      kRobotToLeftCam);

  PhotonPoseEstimator photonEstimatorRight = new PhotonPoseEstimator(
      kTagLayout,
      PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
      kRobotToRightCam);

  public void updatePositionEstamate() {
  }

  @Override
  public void periodic() {
  }

   /**
     * Returns the latest standard deviations of the estimated pose from {@link
     * #getEstimatedGlobalPose()}, for use with {@link
     * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}. This should
     * only be used when there are targets visible.
     */
  public Matrix<N3, N1> getEstimationStdDevs() {
        return curStdDevs;
    }

    // ----- Simulation

    public void simulationPeriodic(Pose2d robotSimPose) {
        visionSim.update(robotSimPose);
    }

    /** Reset pose history of the robot in the vision system simulation. */
    public void resetSimPose(Pose2d pose) {
        if (Robot.isSimulation()) visionSim.resetRobotPose(pose);
    }

    /** A Field2d for visualizing our robot and objects on the field. */
    public Field2d getSimDebugField() {
        if (!Robot.isSimulation()) return null;
        return visionSim.getDebugField();
    }

    @FunctionalInterface
    public static interface EstimateConsumer {
        public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs);
    }
}
