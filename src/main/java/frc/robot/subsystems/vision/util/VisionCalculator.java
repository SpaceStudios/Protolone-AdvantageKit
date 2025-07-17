// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision.util;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/** Add your docs here. */
public class VisionCalculator {
  public static Matrix<N3, N1> calculateStdDevs(VisionObservation observation) {
    double standardDeviationModifier =
        Math.pow(observation.averageTagDistance, 2) / observation.tagAmount;
    return VecBuilder.fill(
        linearstddev.in(Meters) * standardDeviationModifier,
        linearstddev.in(Meters) * standardDeviationModifier,
        angularstddev.in(Radians) * standardDeviationModifier);
  }
}
