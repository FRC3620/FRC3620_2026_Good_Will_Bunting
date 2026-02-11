// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Fahrenheit;

import java.util.EnumSet;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

import org.tinylog.TaggedLogger;
import org.usfirst.frc3620.logger.LoggingMaster;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.core.CoreTalonFX;

import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class HealthSubsystem extends SubsystemBase {
  TaggedLogger logger = LoggingMaster.getLogger(getClass());

  public enum HealthOptions {
  }

  public final static EnumSet<HealthOptions> healthOptionsForYAMS = EnumSet.noneOf(HealthOptions.class);
  public final static EnumSet<HealthOptions> healthOptionsForCTRESwerveMotors = EnumSet.noneOf(HealthOptions.class);
  public final static EnumSet<HealthOptions> healthOptionsForCTRESwerveSensors = EnumSet.noneOf(HealthOptions.class);

  public enum Health {
    GOOD, MEDIOCRE, BAD, DEATHROW;

    public Health worstOf(Health h) {
      if (this.compareTo(h) > 0) return this;
      return h;
    }
  }

  Health currentHealth = Health.DEATHROW;

  public Health getCurrentHealth() {
    return currentHealth;
  }

  Set<CoreTalonFX> all_Fxs = new HashSet<>();
  Set<ParentDevice> all_ctre = new HashSet<>();

  Map<Object, String> all_device_names = new HashMap<>();
  Map<Object, String> all_device_descriptions = new HashMap<>();
  Map<Object, EnumSet<HealthOptions>> all_health_options = new HashMap<>();

  Timer timer = new Timer();

  /** Creates a new HealthSubsystem. */
  public HealthSubsystem() {
    timer.reset();
    timer.start();
  }

  @Override
  public void periodic() {

    Health newHealth = Health.GOOD;

    int numberOfMissingMotors = RobotContainer.canDeviceFinder.getMissingDeviceSet().size();
    if (numberOfMissingMotors > 0) {
      newHealth = Health.MEDIOCRE;
    }

    if (timer.advanceIfElapsed(0.5)) {
      Health talonTemperatureHealth = checkTalonTemperatures();
      newHealth = newHealth.worstOf(talonTemperatureHealth);

      Health ctreConnectionHealth = checkCTREconnections();
      newHealth = newHealth.worstOf(ctreConnectionHealth);

      /*
       * Check questnav and limelights
       */
    }

    // all done, save the result
    currentHealth = newHealth;
  }

  public Health checkTalonTemperatures() {
    Health rv = Health.GOOD;
    for (var device : all_Fxs) {
      var healthOptionsForDevice = all_health_options.get(device);
      var deviceName = all_device_names.get(device);

      StatusSignal<Temperature> tempuratreSignal = device.getDeviceTemp();
      var tempuratre = tempuratreSignal.getValue();

      double tempuratreFahrenheit = tempuratre.in(Fahrenheit);
      if (tempuratreFahrenheit > 90) {
        rv = Health.BAD;
      }
    }
    return rv;
  }

  public Health checkCTREconnections() {
    Health rv = Health.GOOD;
    for (var device : all_ctre) {
      var healthOptionsForDevice = all_health_options.get(device);
      var deviceName = all_device_names.get(device);

      boolean isConnected = device.isConnected();
      if (!isConnected) {
        rv = Health.BAD;
      }
    }
    return rv;
  }

  public void addMotorToWatch(CoreTalonFX device, String name, EnumSet<HealthOptions> healthOptions) {
    all_Fxs.add(device);
    all_ctre.add(device);
    all_device_names.put(device, name);
    all_device_descriptions.put(device, deviceDescription(device));
    all_health_options.put(device, healthOptions);
  }

  public void addCTRESensorToWatch(ParentDevice device, String name, EnumSet<HealthOptions> healthOptions) {
    all_ctre.add(device);
    all_device_names.put(device, name);
    all_device_descriptions.put(device, deviceDescription(device));
    all_health_options.put(device, healthOptions);
  }

  String deviceDescription(Object device) {
    String device_description = device.getClass().getSimpleName();
    if (device instanceof ParentDevice) {
      device_description = device_description + " " + ((ParentDevice) device).getDeviceID();
    }
    return device_description;
  }

  public void dumpDatabase() {
    for (var device_name_entry : all_device_names.entrySet()) {
      Object device = device_name_entry.getKey();
      String device_name = device_name_entry.getValue();
      String device_description = all_device_descriptions.get(device);
      Object health_options = all_health_options.get(device);
      logger.info("{} is {}, options {}", device_name, device_description, health_options);
    }
  }
}
