// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class Vision extends SubsystemBase {
  PhotonCamera leftCamera = new PhotonCamera("OV-Face-Up");
  PhotonCamera rightCamera = new PhotonCamera("AR-Face-Up");

  public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  public static final Transform3d kRobotToLeftCam = new Transform3d(
      new Translation3d(0.5, 0.0, 0.5),
      new Rotation3d(0, 0, 0)
  );

  public static final Transform3d kRobotToRightCam = new Transform3d(
      new Translation3d(0.5, 0.0, 0.5),
      new Rotation3d(0, 0, 0)
  );

  PhotonPoseEstimator photonEstimatorLeft = new PhotonPoseEstimator(
    kTagLayout,
    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
    kRobotToLeftCam
  );

  PhotonPoseEstimator photonEstimatorRight = new PhotonPoseEstimator(
    kTagLayout,
    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
    kRobotToRightCam
  );

  public Vision() {
  }

  public void updatePositionEstamate() {}

  @Override
  public void periodic() {
  }
}
