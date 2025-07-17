// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.VisionConstants;
import java.util.function.DoubleSupplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveAndAimAtTag extends Command {
  /** Creates a new DriveAndAimAtPose. */
  DoubleSupplier joystickX;

  DoubleSupplier joystickY;
  Pose2d targetPose;
  Pose2d currentPose;
  Drive drive;

  public DriveAndAimAtTag(
      DoubleSupplier joystickX, DoubleSupplier joystickY, int tagID, Drive drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.joystickX = joystickX;
    this.joystickY = joystickY;
    if (VisionConstants.fieldLayout.getTagPose(tagID).isPresent()) {
      targetPose = VisionConstants.fieldLayout.getTagPose(tagID).get().toPose2d();
    } else {
      targetPose = new Pose2d();
    }
    this.drive = drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.runVelocity(new ChassisSpeeds());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentPose = drive.getPose();
    Rotation2d angleToPose =
        Rotation2d.fromRadians(
            Math.atan(
                (targetPose.getX() - currentPose.getX())
                    / (targetPose.getY() - currentPose.getY())));
    double radianDifference = angleToPose.getRadians() - currentPose.getRotation().getRadians();
    ChassisSpeeds designatedSpeeds =
        new ChassisSpeeds(
            MathUtil.applyDeadband(joystickX.getAsDouble(), DriveCommands.DEADBAND)
                * drive.getMaxLinearSpeedMetersPerSec(),
            MathUtil.applyDeadband(joystickY.getAsDouble(), DriveCommands.DEADBAND)
                * drive.getMaxLinearSpeedMetersPerSec(),
            radianDifference);
    drive.runVelocity(designatedSpeeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.runVelocity(new ChassisSpeeds());
    currentPose = new Pose2d();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
