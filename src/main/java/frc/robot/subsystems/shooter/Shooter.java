// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.io.MotorDataAutoLogged;
import frc.robot.subsystems.shooter.io.ShooterIO;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  ShooterIO io;

  public MotorDataAutoLogged shooterData;
  public MotorDataAutoLogged launcherData;

  public Shooter(ShooterIO io) {
    this.io = io;
    shooterData = new MotorDataAutoLogged();
    launcherData = new MotorDataAutoLogged();
  }

  public void setShooterVolts(Voltage volts) {
    io.setShooterVolts(volts);
  }

  public void setLauncherVolts(Voltage volts) {
    io.setLauncherVolts(volts);
  }

  @Override
  public void periodic() {
    io.updateInputs(shooterData, launcherData);
    Logger.processInputs("Shooter/Shooter Motor", shooterData);
    Logger.processInputs("Shooter/Launcher Motor", launcherData);
  }
}
