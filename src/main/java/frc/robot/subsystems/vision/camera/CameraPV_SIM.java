// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision.camera;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.vision.util.VisionObservationAutoLogged;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/** Add your docs here. */
public class CameraPV_SIM extends CameraBase {
  PhotonCameraSim camera;
  PhotonPoseEstimator poseEstimator;
  VisionObservationAutoLogged latestObservation;
  PhotonTrackedTarget bestTarget;

  public CameraPV_SIM(String cameraName, Transform3d cameraPosition) {
    camera = new PhotonCameraSim(new PhotonCamera(cameraName), SimCameraProperties.PERFECT_90DEG());
    poseEstimator = new PhotonPoseEstimator(fieldLayout, primaryPoseStrategy, cameraPosition);
    poseEstimator.setMultiTagFallbackStrategy(backupPoseStratgy);
    latestObservation = new VisionObservationAutoLogged();
  }

  public void setSimSystem(VisionSystemSim simSystem) {
    simSystem.addCamera(camera, poseEstimator.getRobotToCameraTransform());
  }

  @Override
  public double getResultTimestamp() {
    return latestObservation.timestampSeconds;
  }

  @Override
  public VisionObservationAutoLogged getLatestObservation() {
    return latestObservation;
  }

  @Override
  public Pose2d getLatestPose() {
    return latestObservation.estimatedPose;
  }

  @Override
  public PhotonTrackedTarget getBestTargetData() {
    return bestTarget;
  }

  @Override
  public void sendHeadingData(double timestamp, Rotation2d headingData) {
    poseEstimator.addHeadingData(getResultTimestamp(), headingData);
  }

  @Override
  public void periodic() {
    List<PhotonPipelineResult> results = camera.getCamera().getAllUnreadResults();
    if (!results.isEmpty()) {
      PhotonPipelineResult result = results.get(0);
      if (result.hasTargets()) {
        latestObservation.timestampSeconds = result.getTimestampSeconds();
        Optional<EstimatedRobotPose> estimatedPose = poseEstimator.update(result);
        bestTarget = result.getBestTarget();
        if (result.multitagResult.isPresent()) {
          latestObservation.ambiguity = result.multitagResult.get().estimatedPose.ambiguity;
        } else {
          latestObservation.ambiguity = result.getBestTarget().poseAmbiguity;
        }
        if (estimatedPose.isPresent()) {
          Pose3d estimated3DPose = estimatedPose.get().estimatedPose;
          latestObservation.estimatedPose = estimated3DPose.toPose2d();
          latestObservation.tagAmount = estimatedPose.get().targetsUsed.size();
          double averageDistance = 0.0;
          for (int i = 0; i < latestObservation.tagAmount; i++) {
            Pose3d tagPose =
                fieldLayout.getTagPose(estimatedPose.get().targetsUsed.get(i).fiducialId).get();
            averageDistance += estimated3DPose.relativeTo(tagPose).getTranslation().getNorm();
          }
          averageDistance /= latestObservation.tagAmount;
          latestObservation.averageTagDistance = averageDistance;
        }
      }
    }
  }
}
