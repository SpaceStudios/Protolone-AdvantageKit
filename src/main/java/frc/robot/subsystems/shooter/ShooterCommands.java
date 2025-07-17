// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/** Add your docs here. */
public class ShooterCommands {
  public static Command setShooterVolts(Voltage volts, Shooter shooter) {
    return Commands.run(
            () -> {
              shooter.setLauncherVolts(volts);
            })
        .finallyDo(
            () -> {
              shooter.setShooterVolts(Volts.zero());
            });
  }

  public static Command setLauncherVolts(Voltage volts, Shooter shooter) {
    return Commands.run(
            () -> {
              shooter.setLauncherVolts(volts);
            })
        .finallyDo(
            () -> {
              shooter.setLauncherVolts(Volts.zero());
            });
  }

  private static Command waitCommand() {
    return Commands.run(() -> {});
  }

  public static Command shooterCommand(
      Voltage volts, Shooter shooter, double secondsBetweenLaunch) {
    return Commands.parallel(
        setLauncherVolts(volts, shooter),
        waitCommand().withTimeout(secondsBetweenLaunch).andThen(setShooterVolts(volts, shooter)));
  }

  public static Command shooterCommand(
      Voltage volts, Shooter shooter, LinearVelocity desiredVelocity) {
    return Commands.parallel(
            setLauncherVolts(volts, shooter),
            waitCommand()
                .until(
                    () ->
                        (desiredVelocity.in(MetersPerSecond)
                            == (shooter.launcherData.velocity.in(RadiansPerSecond)
                                * ShooterConstants.launcherWheelRadius.in(Meters))))
                .andThen(setShooterVolts(volts, shooter)))
        .finallyDo(
            () -> {
              shooter.setLauncherVolts(Volts.zero());
              shooter.setShooterVolts(Volts.zero());
            });
  }
}
