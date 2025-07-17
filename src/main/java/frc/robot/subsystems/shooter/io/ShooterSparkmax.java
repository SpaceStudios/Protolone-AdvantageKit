// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.io;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class ShooterSparkmax extends SubsystemBase implements ShooterIO {
  SparkMax shooterMax;
  SparkMax launcherMax;
  RelativeEncoder shooterEncoder;
  RelativeEncoder launcherEncoder;

  public ShooterSparkmax() {
    shooterMax = new SparkMax(12, MotorType.kBrushless);
    launcherMax = new SparkMax(11, MotorType.kBrushless);

    SparkMaxConfig generalConfig = new SparkMaxConfig();
    generalConfig.smartCurrentLimit(40);
    shooterMax.configure(
        generalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    launcherMax.configure(
        generalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    shooterEncoder = shooterMax.getEncoder();
    launcherEncoder = launcherMax.getEncoder();
  }

  @Override
  public void setShooterVolts(Voltage volts) {
    shooterMax.setVoltage(volts);
  }

  @Override
  public void setLauncherVolts(Voltage volts) {
    launcherMax.setVoltage(volts);
  }

  @Override
  public void updateInputs(MotorDataAutoLogged shooterData, MotorDataAutoLogged launcherData) {
    shooterData.connected = true;
    launcherData.connected = true;

    // Shooter Data
    shooterData.current = Amps.of(shooterMax.getOutputCurrent());
    shooterData.output = shooterMax.get();
    shooterData.temperature = Celsius.of(shooterMax.getMotorTemperature());
    shooterData.velocity = RPM.of(shooterEncoder.getVelocity());

    // Launcher Data
    launcherData.current = Amps.of(launcherMax.getOutputCurrent());
    launcherData.output = launcherMax.get();
    launcherData.temperature = Celsius.of(launcherMax.getMotorTemperature());
    launcherData.velocity = RPM.of(launcherEncoder.getVelocity());
  }
}
