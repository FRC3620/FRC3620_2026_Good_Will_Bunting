// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Fahrenheit;

import java.util.*;
import java.util.function.BooleanSupplier;

import org.tinylog.TaggedLogger;
import org.usfirst.frc3620.CANDeviceType;
import org.usfirst.frc3620.Utilities.GlobMatcher;
import org.usfirst.frc3620.logger.LoggingMaster;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.core.CoreTalonFX;

import edu.wpi.first.hal.PowerDistributionFaults;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotController.RadioLEDState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Elastic;
import frc.robot.RobotContainer;

public class HealthSubsystem extends SubsystemBase {

  TaggedLogger logger = LoggingMaster.getLogger(getClass());

  private final static String alertGroupName = "Health Alerts";

  RadioLEDState radioLEDState = null;

  Health currentHealth = Health.INVALID;

  public Health getCurrentHealth() {
    return currentHealth;
  }

  CTREHealthChecker ctreHealthChecker;
  MotorTemperatureMotorChecker motorTemperatureMotorChecker;
  CircuitBreakerChecker circuitBreakerChecker;
  YesNoHealthChecker genericChecker;

  Timer timer = new Timer();

  /** Creates a new HealthSubsystem. */
  public HealthSubsystem() {
    timer.reset();
    timer.start();

    ctreHealthChecker = new CTREHealthChecker();
    motorTemperatureMotorChecker = new MotorTemperatureMotorChecker();
    genericChecker = new YesNoHealthChecker();
    genericChecker.addIgnoreGlobs(RobotContainer.robotParameters.getIgnoreHealth());

    if (RobotContainer.powerDistribution != null) {
      circuitBreakerChecker = new CircuitBreakerChecker();
    }
  }

  Health missingDeviceHealth = null;
  Health talonTemperatureHealth = Health.GOOD;
  Health ctreConnectionHealth = Health.GOOD;
  Health booleanSupplierHealth = Health.GOOD;
  Health circuitBreakerHealth = Health.GOOD;

  boolean didInfoNotification = false;
  boolean didWarningNotification = false;
  boolean didErrorNotification = false;

  @Override
  public void periodic() {
    Health newHealth = Health.GOOD;

    // only need to do this once
    if (missingDeviceHealth == null) {
      missingDeviceHealth = checkForMissingDevices();
      SmartDashboard.putString("Health/Missing Devices", missingDeviceHealth.toString());
    }
    newHealth = newHealth.worstOf(missingDeviceHealth);

    if (timer.advanceIfElapsed(0.5)) {
      // only check these a couple times a second
      talonTemperatureHealth = checkTalonTemperatures();
      SmartDashboard.putString("Health/Talon Temperature", talonTemperatureHealth.toString());
      ctreConnectionHealth = checkCTREconnections();
      SmartDashboard.putString("Health/CTRE Connections", ctreConnectionHealth.toString());
      circuitBreakerHealth = checkCircuitBreakers();
      SmartDashboard.putString("Health/Circuit Breakers", circuitBreakerHealth.toString());
    }
    newHealth = newHealth.worstOf(talonTemperatureHealth);
    newHealth = newHealth.worstOf(ctreConnectionHealth);
    newHealth = newHealth.worstOf(circuitBreakerHealth);

    booleanSupplierHealth = checkBooleanSuppliers();
    SmartDashboard.putString("Health/Misc", booleanSupplierHealth.toString());
    newHealth = newHealth.worstOf(booleanSupplierHealth);

    if (newHealth != currentHealth) {
      if (newHealth == Health.GOOD) {
        if (! didInfoNotification) {
          Elastic.Notification notification = new Elastic.Notification(Elastic.NotificationLevel.INFO,
              "Info",
              "The current health of the robot has changed. The new health is " + newHealth);
          Elastic.sendNotification(notification.withDisplaySeconds(10.0));
          didInfoNotification = true;
        }
      } else if (newHealth == Health.MEDIOCRE) {
        if (! didWarningNotification) {
          Elastic.Notification notification = new Elastic.Notification(Elastic.NotificationLevel.WARNING,
              "Warning",
              "The current health of the robot has changed. The new health is " + newHealth);
          Elastic.sendNotification(notification.withNoAutoDismiss());
          didWarningNotification = true;
        }
      } else {
        if (! didErrorNotification) {
          Elastic.Notification notification = new Elastic.Notification(Elastic.NotificationLevel.ERROR,
              "ERRORCODE:404",
              "The current health of the robot has changed. The new health is " + newHealth);
          Elastic.sendNotification(notification.withNoAutoDismiss());
          didErrorNotification = true;
        }
      }
    }
    currentHealth = newHealth;
    SmartDashboard.putString("Health/Overall", currentHealth.toString());
    SmartDashboard.putBoolean("Health/Ok", currentHealth == Health.GOOD);

    RadioLEDState newRadioLEDState = null;
    switch (newHealth) {
      case GOOD:
        newRadioLEDState = RadioLEDState.kGreen;
        break;

      case MEDIOCRE:
        newRadioLEDState = RadioLEDState.kOrange;
        break;

      case BAD:
        newRadioLEDState = RadioLEDState.kRed;
        break;

      case DEATHROW:
        newRadioLEDState = ((RobotController.getFPGATime() % 200000) > 100000) ? RadioLEDState.kRed
            : RadioLEDState.kOff;
        break;

      default:
        newRadioLEDState = ((RobotController.getFPGATime() % 200000) > 100000) ? RadioLEDState.kRed
            : RadioLEDState.kOrange;
    }

    if (newRadioLEDState != radioLEDState) {
      RobotController.setRadioLEDState(newRadioLEDState);
      radioLEDState = newRadioLEDState;
    }
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

      boolean competition_robot = RobotContainer.amIACompBot();
      for (var deviceName : missingDeviceNames) {
        if (competition_robot) {
          criticalDeviceNames.add(deviceName);
        } else {
          if (globMatcher.matches(deviceName)) {
            nonCriticalDeviceNames.add(deviceName);
          } else {
            criticalDeviceNames.add(deviceName);
          }
        }
      }

      if (nonCriticalDeviceNames.size() > 0) {
        String text = namesForAlert("Missing from CAN bus, but we don't care", nonCriticalDeviceNames);
        @SuppressWarnings("resource")
        Alert alert = new Alert(alertGroupName, text, AlertType.kInfo);
        alert.set(true);
      }

      if (criticalDeviceNames.size() > 0) {
        rv = competition_robot ? Health.BAD : Health.MEDIOCRE;
        String text = namesForAlert("Missing from CAN bus", criticalDeviceNames);
        @SuppressWarnings("resource")
        Alert alert = new Alert(alertGroupName, text, competition_robot ? AlertType.kError : AlertType.kWarning);
        alert.set(true);
      }
    }
    return rv;
  }

  Health checkTalonTemperatures() {
    boolean ok = motorTemperatureMotorChecker.allAreOk();
    Health rv = ok ? Health.GOOD : Health.BAD;
    return rv;
  }

  Health checkCTREconnections() {
    boolean ok = ctreHealthChecker.allAreOk();
    Health rv = ok ? Health.GOOD : Health.BAD;
    return rv;
  }

  Health checkBooleanSuppliers() {
    boolean ok = genericChecker.allAreOk();
    Health rv = ok ? Health.GOOD : Health.BAD;
    return rv;
  }

  Health checkCircuitBreakers() {
    if (circuitBreakerChecker == null)
      return Health.GOOD;
    boolean ok = circuitBreakerChecker.allAreOk();
    Health rv = ok ? Health.GOOD : Health.MEDIOCRE;
    return rv;
  }

  class YesNoHealthChecker {
    Set<BooleanSupplier> all_booleanSuppliers = new HashSet<>();
    Map<BooleanSupplier, String> all_names = new HashMap<>();
    Map<BooleanSupplier, HealthOptions> all_healthOptions = new HashMap<>();
    Set<BooleanSupplier> ill_booleanSuppliers = new HashSet<>();
    Map<BooleanSupplier, Alert> all_alerts = new HashMap<>();

    GlobMatcher thingsToIgnore = new GlobMatcher();

    void addIgnoreGlob(String glob) {
      thingsToIgnore.addGlob(glob);
    }

    void addIgnoreGlobs(Collection<String> globs) {
      thingsToIgnore.addGlobs(globs);
    }

    void add(BooleanSupplier booleanSupplier, String name, HealthOptions healthOptions) {
      all_booleanSuppliers.add(booleanSupplier);
      all_names.put(booleanSupplier, name);
      all_healthOptions.put(booleanSupplier, healthOptions);

      if (healthOptions.shouldShowAlertWhenBad()) {
        if (thingsToIgnore.matches(name)) {
          all_alerts.put(booleanSupplier, new Alert(alertGroupName, name + ", but we don't care", AlertType.kWarning));
        } else {
          all_alerts.put(booleanSupplier, new Alert(alertGroupName, name, AlertType.kError));
        }
      }
    }

    // override these if you need to do something before or after the allAreOk loop
    boolean prolog() {
      return true;
    }

    boolean epilog(boolean ok) {
      return ok;
    }

    boolean allAreOk() {
      boolean ok = prolog();
      ill_booleanSuppliers.clear();
      for (var booleanSupplier : all_booleanSuppliers) {
        // HealthOptions healthOptions = all_healthOptions.get(booleanSupplier);
        String name = all_names.get(booleanSupplier);
        Alert alert = all_alerts.get(booleanSupplier);

        boolean isOk = booleanSupplier.getAsBoolean();

        if (isOk) {
          if (alert != null) {
            alert.set(false);
          }

        } else {
          if (alert != null) {
            alert.set(true);
          }
          if (!thingsToIgnore.matches(name)) {
            ok = false;
          }
        }
      }
      ok = epilog(ok);
      return ok;
    }

    String getAllIllSupplierNamesText(String heading) {
      return namesForAlert(heading, getAllIllSupplierNames());
    }

    List<String> getAllIllSupplierNames() {
      List<String> rv = new ArrayList<>(ill_booleanSuppliers.size());
      for (var bs : ill_booleanSuppliers) {
        rv.add(all_names.get(bs));
      }
      return rv;

    }
  }

  class MotorTemperatureMotorChecker extends YesNoHealthChecker {
    void addMotor(CoreTalonFX device, String name, HealthOptions healthOptions) {
      Temperature threshold = healthOptions.getMotorTemperatureThreshold();

      BooleanSupplier bs = () -> {
        StatusSignal<Temperature> device_temperature_status = device.getDeviceTemp();
        Temperature device_temperature = device_temperature_status.getValue();
        boolean ok = device_temperature.lt(threshold);
        return ok;
      };

      add(bs, name, healthOptions);
    }
  }

  class CTREHealthChecker extends YesNoHealthChecker {
    void addDevice(ParentDevice device, String name, HealthOptions healthOptions) {
      BooleanSupplier booleanSupplier = () -> {
        boolean ok = device.isConnected();
        return ok;
      };
      add(booleanSupplier, name, healthOptions);
    }
  }

  class CircuitBreakerChecker extends YesNoHealthChecker {
    PowerDistribution powerDistribution;
    Alert alert = new Alert(alertGroupName, "", AlertType.kWarning);
    Set<Integer> breakersToIgnore = new HashSet<>(RobotContainer.robotParameters.getBreakersToIgnore());

    CircuitBreakerChecker() {
      super();
      powerDistribution = RobotContainer.powerDistribution;
      add(() -> isOk(), "PDH", new HealthOptions());
    }

    String lastAlertText = "";

    public boolean isOk() {
      boolean ok = true;
      PowerDistributionFaults faults = powerDistribution.getFaults();
      Set<String> breakerFaults = new HashSet<>();
      for (int channel = 0; channel < 24; channel++) {
        if (faults.getBreakerFault(channel) && !breakersToIgnore.contains(channel)) {
          ok = false;
          breakerFaults.add("Breaker " + channel);
        }
      }
      if (faults.CanWarning) {
        ok = false;
        breakerFaults.add("CAN warning");
      }
      if (faults.HardwareFault) {
        ok = false;
        breakerFaults.add("Hardware Fault");
      }

      if (!ok) {
        String alertText = namesForAlert("PDH issue(s):", breakerFaults);
        if (!lastAlertText.equals(alertText)) {
          alert.setText(alertText);
          lastAlertText = alertText;
        }
        alert.set(true);
      } else {
        alert.set(false);
      }
      return ok;
    }
  }

  public void addMotorToWatch(CoreTalonFX device, String name, HealthOptions healthOptions) {
    motorTemperatureMotorChecker.addMotor(device, name + " is hot!", healthOptions);
    ctreHealthChecker.addDevice(device, name + " is disconnected!", healthOptions);
  }

  public void addCTRESensorToWatch(ParentDevice device, String name, HealthOptions healthOptions) {
    ctreHealthChecker.addDevice(device, name + " is disconnected!", healthOptions);
  }

  public void addHealthyBooleanSupplier(BooleanSupplier booleanSupplier, String name, HealthOptions healthOptions) {
    genericChecker.add(booleanSupplier, name, healthOptions);
  }

  String deviceDescription(Object device) {
    String device_description = device.getClass().getSimpleName();
    if (device instanceof ParentDevice) {
      device_description = device_description + " " + ((ParentDevice) device).getDeviceID();
    }
    return device_description;
  }

  public void dumpDatabase() {
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

  public static class HealthOptions {
    // if this is set, then put the device temperature into the network tables
    private boolean sendTemperatureToNT = false;

    // if this is set, then put changes from health to sick into the TaggedLogger
    private boolean sendHealthChangesToTinyLog = false;

    // temperature that we get nervous about motor temps at
    private Temperature motorTemperatureThreshold = Celsius.of(55);

    // we should put up an Alert if things go South
    private boolean showAlertWhenBad = false;

    public HealthOptions() {
    }

    public HealthOptions(HealthOptions template) {
      this();
      // copy fields over
      sendTemperatureToNT = template.sendTemperatureToNT;
      sendHealthChangesToTinyLog = template.sendHealthChangesToTinyLog;
      motorTemperatureThreshold = template.motorTemperatureThreshold;
      showAlertWhenBad = template.showAlertWhenBad;
    }

    public HealthOptions withSendTemperatureToNT(boolean v) {
      HealthOptions rv = new HealthOptions(this);
      rv.sendTemperatureToNT = v;
      return rv;
    }

    public boolean getSendTemperatureToNT() {
      return sendTemperatureToNT;
    }

    public HealthOptions withSendHealthChangesToTinylog(boolean v) {
      HealthOptions rv = new HealthOptions(this);
      rv.sendHealthChangesToTinyLog = v;
      return rv;
    }

    public boolean getSendHealthChangesToTinyLog() {
      return sendHealthChangesToTinyLog;
    }

    public HealthOptions withMotorTemperatureThreshold(Temperature threshold) {
      HealthOptions rv = new HealthOptions(this);
      rv.motorTemperatureThreshold = threshold;
      return rv;
    }

    public Temperature getMotorTemperatureThreshold() {
      return motorTemperatureThreshold;
    }

    public HealthOptions withShowAlertWhenBad(boolean b) {
      HealthOptions rv = new HealthOptions(this);
      rv.showAlertWhenBad = b;
      return rv;
    }

    public boolean shouldShowAlertWhenBad() {
      return showAlertWhenBad;
    }

    @Override
    public String toString() {
      return "HealthOptions [sendTemperatureToNT=" + sendTemperatureToNT + ", sendHealthChangesToTinyLog="
          + sendHealthChangesToTinyLog + ", motorTemperatureThreshold=" + motorTemperatureThreshold
          + ", showAlertWhenBad=" + showAlertWhenBad + "]";
    }
  }

  public final static HealthOptions healthOptionsForYAMS = new HealthOptions().withShowAlertWhenBad(true);
  public final static HealthOptions healthOptionsForCTRESwerveMotors = new HealthOptions().withShowAlertWhenBad(true);
  public final static HealthOptions healthOptionsForCTRESwerveSensors = new HealthOptions().withShowAlertWhenBad(true);

  public enum Health {
    INVALID, GOOD, MEDIOCRE, BAD, DEATHROW;

    public Health worstOf(Health h) {
      if (this.compareTo(h) > 0)
        return this;
      return h;
    }
  }

}
