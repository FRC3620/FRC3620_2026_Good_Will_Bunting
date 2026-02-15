// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Fahrenheit;

import java.util.*;
import java.util.function.BooleanSupplier;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.util.regex.PatternSyntaxException;

import org.tinylog.TaggedLogger;
import org.usfirst.frc3620.Utilities;
import org.usfirst.frc3620.CANDeviceFinder.NamedCANDevice;
import org.usfirst.frc3620.Utilities.GlobMatcher;
import org.usfirst.frc3620.logger.LoggingMaster;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.core.CoreTalonFX;

import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class HealthSubsystem extends SubsystemBase {
  TaggedLogger logger = LoggingMaster.getLogger(getClass());

  private final static String alertGroupName = "Health Alerts";

  public enum Health {
    GOOD, MEDIOCRE, BAD, DEATHROW;

    public Health worstOf(Health h) {
      if (this.compareTo(h) > 0)
        return this;
      return h;
    }
  }

  Health currentHealth = Health.DEATHROW;

  public Health getCurrentHealth() {
    return currentHealth;
  }

  Set<CoreTalonFX> all_Fxs = new HashSet<>();
  Set<ParentDevice> all_ctre = new HashSet<>();
  Set<BooleanSupplier> all_booleanSuppliers = new HashSet<>();

  Map<Object, String> all_device_names = new HashMap<>();
  Map<Object, String> all_device_descriptions = new HashMap<>();
  Map<Object, HealthOptions> all_health_options = new HashMap<>();

  Timer timer = new Timer();

  /** Creates a new HealthSubsystem. */
  public HealthSubsystem() {
    timer.reset();
    timer.start();

    booleanSupplierIgnores = new GlobMatcher(RobotContainer.robotParameters.getIgnoreHealth());
  }

  Health missingDeviceHealth = null;
  Health talonTemperatureHealth = Health.GOOD;
  Map<Object, Alert> talonTemperatureAlerts = new HashMap<>();
  Health ctreConnectionHealth = Health.GOOD;
  Map<Object, Alert> ctreConnectionAlerts = new HashMap<>();
  Health booleanSupplierHealth = Health.GOOD;
  Map<Object, Alert> booleanSupplierAlerts = new HashMap<>();
  GlobMatcher booleanSupplierIgnores;

  @Override
  public void periodic() {
    Health newHealth = Health.GOOD;

    // only need to do this once
    if (missingDeviceHealth == null) {
      missingDeviceHealth = checkForMissingDevices();
    }
    newHealth = newHealth.worstOf(missingDeviceHealth);

    if (timer.advanceIfElapsed(0.5)) {
      // only check these a couple times a second
      talonTemperatureHealth = checkTalonTemperatures();
      ctreConnectionHealth = checkCTREconnections();
    }
    newHealth = newHealth.worstOf(talonTemperatureHealth);
    newHealth = newHealth.worstOf(ctreConnectionHealth);

    booleanSupplierHealth = checkBooleanSuppliers();
    newHealth = newHealth.worstOf(booleanSupplierHealth);

    currentHealth = newHealth;
  }

  Health checkForMissingDevices() {
    Health rv = Health.GOOD;
    var missingDevices = RobotContainer.canDeviceFinder.getMissingDeviceSet();
    if (missingDevices.size() > 0) {
      GlobMatcher globMatcher = new GlobMatcher(RobotContainer.robotParameters.getNonCriticalCANDevices());

      List<String> missingDeviceNames = new ArrayList<>();
      for (var missingDevice : missingDevices) {
        missingDeviceNames.add(missingDevice.toString());
      }
      Collections.sort(missingDeviceNames);

      List<String> criticalDeviceNames = new ArrayList<>();
      List<String> nonCriticalDeviceNames = new ArrayList<>();

      for (var deviceName : missingDeviceNames) {
        if (globMatcher.matches(deviceName)) {
          nonCriticalDeviceNames.add(deviceName);
        } else {
          criticalDeviceNames.add(deviceName);
        }
      }

      if (nonCriticalDeviceNames.size() > 0) {
        String text = namesForAlert("Missing from CAN bus, but we don't care", nonCriticalDeviceNames);
        @SuppressWarnings("resource")
        Alert alert = new Alert(alertGroupName, text, AlertType.kInfo);
        alert.set(true);
      }

      if (criticalDeviceNames.size() > 0) {
        rv = Health.MEDIOCRE;
        String text = namesForAlert("Missing from CAN bus", criticalDeviceNames);
        @SuppressWarnings("resource")
        Alert alert = new Alert(alertGroupName, text, AlertType.kWarning);
        alert.set(true);
      }
    }
    return rv;
  }

  String namesForAlert(String heading, Collection<String> names) {
    StringBuilder sb = new StringBuilder();
    if (heading != null) {
      sb.append(heading);
    }
    for (var name : names) {
      if (sb.length() > 0)
        sb.append("\n");
      sb.append(name);
    }
    return sb.toString();
  }

  Health checkTalonTemperatures() {
    Health rv = Health.GOOD;
    for (var device : all_Fxs) {
      // var healthOptionsForDevice = all_health_options.get(device);
      // var deviceName = all_device_names.get(device);
      var alert = talonTemperatureAlerts.get(device);

      StatusSignal<Temperature> tempuratreSignal = device.getDeviceTemp();
      var tempuratre = tempuratreSignal.getValue();

      double tempuratreFahrenheit = tempuratre.in(Fahrenheit);
      if (tempuratreFahrenheit > 90) {
        rv = Health.BAD;
        alert.set(true);
      } else {
        alert.set(false);
      }
    }
    return rv;
  }

  Health checkCTREconnections() {
    Health rv = Health.GOOD;
    for (var device : all_ctre) {
      // var healthOptionsForDevice = all_health_options.get(device);
      // var deviceName = all_device_names.get(device);
      var alert = ctreConnectionAlerts.get(device);

      boolean isOk = device.isConnected();
      if (!isOk) {
        rv = Health.BAD;
        alert.set(true);
      } else {
        alert.set(false);
      }
    }
    return rv;
  }

  public Health checkBooleanSuppliers() {
    Health rv = Health.GOOD;
    for (var device : all_booleanSuppliers) {
      // var healthOptionsForDevice = all_health_options.get(device);
      var alert = booleanSupplierAlerts.get(device);
      var deviceName = all_device_names.get(device);
      boolean isOk = device.getAsBoolean();
      SmartDashboard.putBoolean("Health/" + deviceName + "/healthy", isOk);
      if (!booleanSupplierIgnores.matches(deviceName)) {
        if (!isOk) {
          rv = Health.BAD;
          alert.set(true);
        } else {
          alert.set(false);
        }
      }
    }
    return rv;
  }

  public void addMotorToWatch(CoreTalonFX device, String name, HealthOptions healthOptions) {
    all_Fxs.add(device);
    all_ctre.add(device);
    all_device_names.put(device, name);
    all_device_descriptions.put(device, deviceDescription(device));
    all_health_options.put(device, healthOptions);
    talonTemperatureAlerts.put(device, new Alert(alertGroupName, name + " is hot!", AlertType.kError));
    ctreConnectionAlerts.put(device, new Alert(alertGroupName, name + " is disconnected!", AlertType.kError));
  }

  public void addCTRESensorToWatch(ParentDevice device, String name, HealthOptions healthOptions) {
    all_ctre.add(device);
    all_device_names.put(device, name);
    all_device_descriptions.put(device, deviceDescription(device));
    all_health_options.put(device, healthOptions);
    ctreConnectionAlerts.put(device, new Alert(alertGroupName, name + " is disconnected!", AlertType.kError));
  }

  public void addHealthyBooleanSupplier(BooleanSupplier booleanSupplier, String name, HealthOptions healthOptions) {
    all_booleanSuppliers.add(booleanSupplier);
    all_device_names.put(booleanSupplier, name);
    all_device_descriptions.put(booleanSupplier, deviceDescription(booleanSupplier));
    all_health_options.put(booleanSupplier, healthOptions);
    booleanSupplierAlerts.put(booleanSupplier, new Alert(alertGroupName, name, AlertType.kError));
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

  public static class HealthOptions {
  }

  public final static HealthOptions healthOptionsForYAMS = new HealthOptions();
  public final static HealthOptions healthOptionsForCTRESwerveMotors = new HealthOptions();
  public final static HealthOptions healthOptionsForCTRESwerveSensors = new HealthOptions();
}
