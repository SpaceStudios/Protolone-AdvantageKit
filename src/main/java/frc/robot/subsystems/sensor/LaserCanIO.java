// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.sensor;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;

/** Add your docs here. */
public class LaserCanIO {
    private final LaserCan laserCan;
    private final double threshold;
    public LaserCanIO(int id, double threshold) {
        laserCan = new LaserCan(id);
        this.threshold = threshold;
    }


    public boolean detected() {
        Measurement measurement = laserCan.getMeasurement();
        if (measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            return measurement.distance_mm <= threshold;
        } else {
            return false;
        }
    }
}
