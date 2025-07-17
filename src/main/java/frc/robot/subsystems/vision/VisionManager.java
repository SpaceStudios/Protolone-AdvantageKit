// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;
import static frc.robot.subsystems.vision.util.VisionCalculator.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.vision.camera.CameraBase;
import frc.robot.subsystems.vision.camera.CameraBase.VisionConsumer;
import frc.robot.subsystems.vision.camera.CameraBase.cameraType;
import frc.robot.subsystems.vision.camera.CameraPV;
import frc.robot.subsystems.vision.camera.CameraPV_SIM;
import frc.robot.subsystems.vision.util.VisionObservationAutoLogged;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

/** Add your docs here. */
public class VisionManager extends SubsystemBase {
  CameraBase[] cameras;
  VisionConsumer consumer;
  VisionSystemSim simWorld;
  Supplier<Pose2d> poseGetter;
  Supplier<Rotation2d> headingSupplier;

  /**
   * A simple function that allows me to get the best target from the cameras
   *
   * @param visionMeasurement the function that consumes vision measurements in terms of a Pose2d,
   *     double, and Matrix<N3,N1>
   * @param poseGetter a function that returns the robot's current pose, only used in Simulation not
   *     used anywhere else yet.
   * @param type the cameraType that the robot is using currently only supports PhotonVision and
   *     PhotonVision Sim, Limelight is a work in progress
   * @param headingGetter a function that gets the robot's current heading
   */
  public VisionManager(
      VisionConsumer visionMeasurement,
      Supplier<Pose2d> poseGetter,
      cameraType type,
      Supplier<Rotation2d> headingGetter) {
    if (Robot.isSimulation()) {
      simWorld = new VisionSystemSim("main");
      simWorld.addAprilTags(fieldLayout);
    }
    consumer = visionMeasurement;
    cameras = new CameraBase[cameraNames.length];
    for (int i = 0; i < cameras.length; i++) {
      switch (type) {
        case photonCam:
          cameras[i] = new CameraPV(cameraNames[i], cameraTransforms[i]);
          break;
        case photonSim:
          CameraPV_SIM camSim = new CameraPV_SIM(cameraNames[i], cameraTransforms[i]);
          camSim.setSimSystem(simWorld);
          cameras[i] = camSim;
          break;
        case limelight:
          break;
      }
    }
    this.poseGetter = poseGetter;
    this.headingSupplier = headingGetter;
  }

  @Override
  public void simulationPeriodic() {
    simWorld.update(poseGetter.get());
  }

  /**
   * A simple function that allows me to get the best target from the cameras
   *
   * @return the best target from all the cameras managed by the manager
   */
  public PhotonTrackedTarget[] getBestTargets() {
    PhotonTrackedTarget[] targets = new PhotonTrackedTarget[cameras.length];
    for (int i = 0; i < cameras.length; i++) {
      targets[i] = cameras[i].getBestTargetData();
    }
    return targets;
  }

  /**
   * A simple function that allows me to get the best target from the cameras
   *
   * @param cameraIndex the index of the camera you want to get data from
   * @return the best target from the camera at the specified index
   */
  public PhotonTrackedTarget getBestTarget(int cameraIndex) {
    return cameras[cameraIndex].getBestTargetData();
  }

  @Override
  public void periodic() {
    for (int i = 0; i < cameras.length; i++) {
      VisionObservationAutoLogged observation = cameras[i].getLatestObservation();
      if ((Timer.getFPGATimestamp() - observation.timestampSeconds) < 2
          && observation.estimatedPose != null) {
        // consumer.accept(
        //     observation.estimatedPose, observation.timestampSeconds,
        // calculateStdDevs(observation));
      }
      if (primaryPoseStrategy == PoseStrategy.PNP_DISTANCE_TRIG_SOLVE
          || primaryPoseStrategy == PoseStrategy.PNP_DISTANCE_TRIG_SOLVE) {
        cameras[i].sendHeadingData(Timer.getFPGATimestamp(), headingSupplier.get());
      }
      Logger.processInputs("vision/camera" + i, cameras[i].getLatestObservation());
    }
  }
}
