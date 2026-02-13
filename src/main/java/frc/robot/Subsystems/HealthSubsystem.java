// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Fahrenheit;

import java.util.HashMap;
import java.util.LinkedHashSet;
import java.util.Map;
import java.util.Set;
import java.util.function.BooleanSupplier;

import org.tinylog.TaggedLogger;
import org.usfirst.frc3620.logger.LoggingMaster;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.core.CoreTalonFX;

import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class HealthSubsystem extends SubsystemBase {
  TaggedLogger logger = LoggingMaster.getLogger(getClass());

  private final static String alertGroupName = "Health Alerts";

  public enum Health {
    GOOD, MEDIOCRE, BAD, DEATHROW;

    public Health worstOfThisAnd(Health h) {
      if (this.compareTo(h) > 0)
        return this;
      return h;
    }
  }

  Health currentHealth = Health.DEATHROW;

  public Health getCurrentHealth() {
    return currentHealth;
  }

  Set<CoreTalonFX> all_Fxs = new LinkedHashSet<>();
  Set<ParentDevice> all_ctre = new LinkedHashSet<>();
  Set<BooleanSupplier> all_generics = new LinkedHashSet<>();

  Map<Object, String> all_device_descriptions = new HashMap<>();
  Map<Object, HealthOptions> all_health_options = new HashMap<>();
  Map<Object, Alert> all_alerts = new HashMap<>();

  Timer timer = new Timer();

  /** Creates a new HealthSubsystem. */
  public HealthSubsystem() {
    timer.reset();
    timer.start();

    thingy_timer.reset();
    thingy_timer.start();

    if (Robot.isSimulation()) {
      this.addGeneric(() -> thingy1(), "Thingy1 is busted", new HealthOptions());
      this.addGeneric(() -> thingy2(), "Thingy2 is busted", new HealthOptions());
    }
  }

  Timer thingy_timer = new Timer();

  boolean thingy1() {
    var t = thingy_timer.get();
    var rv = t % 10 > 5;
    return rv;
  }

  boolean thingy2() {
    var t = thingy_timer.get();
    var rv = t % 2 > 1;
    return rv;
  }

  Health missingDeviceHealth = null;

  @Override
  public void periodic() {
    Health newHealth = Health.GOOD;

    // only need to do this once at the beginning
    if (missingDeviceHealth == null) {
      var missingDeviceSet = RobotContainer.canDeviceFinder.getMissingDeviceSet();
      if (missingDeviceSet.isEmpty()) {
        missingDeviceHealth = Health.GOOD;
      } else {
        missingDeviceHealth = Health.MEDIOCRE;
        for (var missingDevice : missingDeviceSet) {
          @SuppressWarnings("resource")
          Alert alert = new Alert(alertGroupName, missingDevice.toString() + " is not on CAN bus", AlertType.kWarning);
          alert.set(true);
        }
      }
    }
    newHealth = newHealth.worstOfThisAnd(missingDeviceHealth);

    if (timer.advanceIfElapsed(0.5)) {
      Health talonTemperatureHealth = checkTalonTemperatures();
      newHealth = newHealth.worstOfThisAnd(talonTemperatureHealth);

      Health ctreConnectionHealth = checkCTREconnections();
      newHealth = newHealth.worstOfThisAnd(ctreConnectionHealth);

      Health genericsHealth = checkGenerics();
      newHealth = newHealth.worstOfThisAnd(genericsHealth);

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
      var alert = all_alerts.get(device);
      var healthOptionsForDevice = all_health_options.get(device);

      StatusSignal<Temperature> tempuratreSignal = device.getDeviceTemp();
      var tempuratre = tempuratreSignal.getValue();

      double tempuratreFahrenheit = tempuratre.in(Fahrenheit);
      if (tempuratreFahrenheit > healthOptionsForDevice.getMotorTemperatureThreshold()) {
        rv = Health.BAD;
        alert.set(true);
      } else {
        alert.set(false);
      }
    }
    return rv;
  }

  public Health checkCTREconnections() {
    Health rv = Health.GOOD;
    for (var device : all_ctre) {
      var alert = all_alerts.get(device);
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

  public Health checkGenerics() {
    Health rv = Health.GOOD;
    for (var device : all_generics) {
      var alert = all_alerts.get(device);
      boolean isOk = device.getAsBoolean();
      if (!isOk) {
        rv = Health.BAD;
        alert.set(true);
      } else {
        alert.set(false);
      }
    }
    return rv;
  }

  public void addMotorToWatch(CoreTalonFX device, String name, HealthOptions healthOptions) {
    all_Fxs.add(device);
    all_ctre.add(device);
    all_device_descriptions.put(device, makeDeviceDescription(name, device));
    all_health_options.put(device, healthOptions);
    Alert alert = new Alert(alertGroupName, name, AlertType.kError);
    alert.set(false);
    all_alerts.put(device, alert);
  }

  public void addCTRESensorToWatch(ParentDevice device, String name, HealthOptions healthOptions) {
    all_ctre.add(device);
    all_device_descriptions.put(device, makeDeviceDescription(name, device));
    all_health_options.put(device, healthOptions);
    Alert alert = new Alert(alertGroupName, name, AlertType.kError);
    alert.set(false);
    all_alerts.put(device, alert);
  }

  public void addGeneric(BooleanSupplier isGood, String name, HealthOptions healthOptions) {
    all_generics.add(isGood);
    all_device_descriptions.put(isGood, makeDeviceDescription(name, null));
    all_health_options.put(isGood, healthOptions);
    Alert alert = new Alert(alertGroupName, name, AlertType.kError);
    alert.set(false);
    all_alerts.put(isGood, alert);
  }

  String makeDeviceDescription(String name, Object device) {
    StringBuffer sb = new StringBuffer(name);
    if (device != null) {
      sb.append(" (");
      sb.append(device.getClass().getSimpleName());
      if (device instanceof ParentDevice) {
        sb.append(" #");
        sb.append(((ParentDevice) device).getDeviceID());
      }
      sb.append(")");
    }
    String s = sb.toString();
    return s;
  }

  public void dumpDatabase() {
    for (var device_description_entry : all_device_descriptions.entrySet()) {
      Object device = device_description_entry.getKey();
      String device_description = all_device_descriptions.get(device);
      Object health_options = all_health_options.get(device);
      logger.info("{}, options {}", device_description, health_options);
    }
  }

  public static class HealthOptions {
    // if this is set, then put the device temperature into the network tables
    private boolean doLogTemperature = false;

    // if this is set, then put changes from health to sick into the TaggedLogger
    private boolean doLogHealthChanges = false;

    // temperature that we get nervous about motor temps at
    private double motorTemperatureThreshold = 120;

    public HealthOptions() {
    }

    public HealthOptions(HealthOptions template) {
      this();
      // copy fields over
      doLogTemperature = template.doLogTemperature;
      doLogHealthChanges = template.doLogHealthChanges;
      motorTemperatureThreshold = template.motorTemperatureThreshold;
    }

    public HealthOptions withDoLogTemperature(boolean v) {
      HealthOptions rv = new HealthOptions(this);
      rv.doLogTemperature = v;
      return rv;
    }

    public boolean getDoLogTemperature() {
      return doLogTemperature;
    }

    public HealthOptions withDoLogHealthChanges(boolean v) {
      HealthOptions rv = new HealthOptions(this);
      rv.doLogHealthChanges = v;
      return rv;
    }

    public boolean getDoLogHealthChanges() {
      return doLogHealthChanges;
    }

    public HealthOptions withMotorTemperatureThreshold(double fahrenheit) {
      HealthOptions rv = new HealthOptions(this);
      rv.motorTemperatureThreshold = fahrenheit;
      return rv;
    }

    public double getMotorTemperatureThreshold() {
      return motorTemperatureThreshold;
    }

    @Override
    public String toString() {
      return "HealthOptions [doLogTemperature=" + doLogTemperature + ", doLogHealthChanges=" + doLogHealthChanges
          + ", motorTemperatureThreshold=" + motorTemperatureThreshold + "]";
    }
  }

  public final static HealthOptions healthOptionsForYAMS = new HealthOptions();
  public final static HealthOptions healthOptionsForCTRESwerveMotors = new HealthOptions().withDoLogTemperature(true);
  public final static HealthOptions healthOptionsForCTRESwerveSensors = new HealthOptions();

}
