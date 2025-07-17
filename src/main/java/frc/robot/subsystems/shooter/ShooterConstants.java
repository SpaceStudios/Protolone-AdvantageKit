// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

/** Add your docs here. */
public class ShooterConstants {
  public static final double secondsBetweenLaunch = 2.0;
  public static final Distance launcherWheelRadius = Inches.of(4).div(2);

  public static Distance calculateLaunchDistance(Distance height, Angle shooterAngle) {
    return Meters.of((Math.tan(shooterAngle.in(Radians))) / height.in(Meters));
  }

  public static Angle calculateLaunchAngle(Distance length, Distance height) {
    return Radians.of(Math.atan(height.in(Meters) / length.in(Meters)));
  }

  public static boolean isInRange(Pose3d robotPose, Pose3d targetPose, Angle shooterAngle) {
    double flatDistance =
        robotPose.toPose2d().relativeTo(targetPose.toPose2d()).getTranslation().getNorm();
    Pose3d relativePose = robotPose.relativeTo(targetPose);
    return (MathUtil.applyDeadband(
            (flatDistance
                - calculateLaunchDistance(relativePose.getMeasureY(), shooterAngle).in(Meters)),
            0.1)
        == 0.0);
  }
}
