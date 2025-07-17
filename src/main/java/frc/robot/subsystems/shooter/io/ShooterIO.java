// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.io;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ShooterIO {
  @AutoLog
  public class MotorData {
    public boolean connected = false;
    public Temperature temperature = Celsius.zero();
    public Current current = Amps.zero();
    public double output = 0.0;
    public AngularVelocity velocity = RadiansPerSecond.zero();
  }

  public default void setShooterVolts(Voltage volts) {}
  ;

  public default void setLauncherVolts(Voltage volts) {}
  ;

  public default void updateInputs(
      MotorDataAutoLogged shooterData, MotorDataAutoLogged launcherData) {}
  ;
}
