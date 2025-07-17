// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision.util;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;

/** A simple storage class */
@AutoLog
public class VisionObservation {
  public double averageTagDistance;
  public int tagAmount;
  public Pose2d estimatedPose;
  public double timestampSeconds;
  public double ambiguity;
}
