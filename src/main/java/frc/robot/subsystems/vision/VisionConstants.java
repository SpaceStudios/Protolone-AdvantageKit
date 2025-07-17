package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class VisionConstants {
  /**
   * The names of the cameras, make sure the position of the names matches the transform camera at
   * the same index or the program will screw up transforms
   */
  public static final String[] cameraNames = new String[] {"CamLeft", "CamRight"};

  /**
   * Transforms of the camera, make sure the position of the transform matches the camera at the
   * same index or the program will screw up transforms
   */
  public static final Transform3d[] cameraTransforms =
      new Transform3d[] {
        new Transform3d(
            Inches.of(13.5),
            Inches.of(13),
            Inches.of(2),
            new Rotation3d(0.0, -Units.degreesToRadians(13.125000), Units.degreesToRadians(-30))),
        new Transform3d(
            Inches.of(13.5),
            Inches.of(-13),
            Inches.of(2),
            new Rotation3d(0.0, -Units.degreesToRadians(13.125000), Units.degreesToRadians(30)))
      };

  // Defining Pose Estimator Stuff
  public static final AprilTagFieldLayout fieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark); // Field Layout
  public static final PoseStrategy primaryPoseStrategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;
  public static final PoseStrategy backupPoseStratgy = PoseStrategy.LOWEST_AMBIGUITY;
  // Standard Deviations Calculator
  public static final Distance linearstddev =
      Meters.of(0.02); // Specify how much the average standard deviation of the linear distance
  public static final Angle angularstddev =
      Radians.of(0.06); // Specify how much the average standard deviation of the linear distance
}
