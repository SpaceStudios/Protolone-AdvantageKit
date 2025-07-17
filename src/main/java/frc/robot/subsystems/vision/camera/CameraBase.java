// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision.camera;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.util.VisionObservationAutoLogged;
import org.photonvision.targeting.PhotonTrackedTarget;

public class CameraBase extends SubsystemBase {

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }

  public enum cameraType {
    photonCam,
    photonSim,
    limelight
  }
  /** Creates a new CameraBase. */
  protected CameraBase() {}

  /**
   * Returns the latest measurement from the camera.
   *
   * @return the latest measurement in the form of a 2d position.
   */
  public Pose2d getLatestPose() {
    return new Pose2d();
  }

  /**
   * Returns the latest observation from the camera.
   *
   * @return the latest observation.
   */
  public VisionObservationAutoLogged getLatestObservation() {
    return new VisionObservationAutoLogged();
  }

  /**
   * Returns the latest observation from the camera.
   *
   * @return the latest observation.
   */
  public PhotonTrackedTarget getBestTargetData() {
    return null;
  }

  /**
   * Returns the timestamp in seconds of the latest result.
   *
   * @return the timestamp of the latest result in seconds
   */
  public double getResultTimestamp() {
    return -1.0;
  }

  /**
   * Sends the heading data to the robot
   *
   * @param headingData the heading of the robot as read from the gyro
   */
  public void sendHeadingData(double timestamp, Rotation2d headingData) {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
