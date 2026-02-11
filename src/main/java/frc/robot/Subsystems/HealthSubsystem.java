// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Fahrenheit;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class HealthSubsystem extends SubsystemBase {
  public enum Health {
    GOOD, MEDIOCRE, BAD, DEATHROW
  }

  Health currentHealth = Health.DEATHROW;

  public Health getCurrentHealth() {
    return currentHealth;
  }

  List<TalonFX> all_Fxs = new ArrayList<>();

  /** Creates a new HealthSubsystem. */
  public HealthSubsystem() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    int numberOfMissingMotors = RobotContainer.canDeviceFinder.getMissingDeviceSet().size();
    if (numberOfMissingMotors == 0) {
      currentHealth = Health.GOOD;
    } else {
      currentHealth = Health.MEDIOCRE;
    }
    ;
    for (TalonFX talonFX : all_Fxs) {
      StatusSignal<Temperature> tempuratreSignal = talonFX.getDeviceTemp();
      var tempuratre = tempuratreSignal.getValue();
      double tempuratreFahrenheit = tempuratre.in(Fahrenheit);
      if (tempuratreFahrenheit > 90) {
        currentHealth = Health.BAD;
      }
      boolean isConnected = talonFX.isConnected();
      if (!isConnected) {
        currentHealth = Health.BAD;
      }
      SmartDashboard.putNumber("Temp", tempuratreFahrenheit);
      SmartDashboard.putBoolean("Connected", isConnected);
    }
    /*
     * Check questnav and limelights
     */
  }

  public void addMotorToWatch(TalonFX talon, String name) {
    all_Fxs.add(talon);
  }
}
