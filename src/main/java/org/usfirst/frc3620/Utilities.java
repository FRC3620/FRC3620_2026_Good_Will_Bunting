// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.usfirst.frc3620;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import java.util.LinkedList;
import java.util.List;
import java.util.function.Consumer;

import org.tinylog.TaggedLogger;
import org.usfirst.frc3620.logger.LoggingMaster;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RuntimeType;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.util.sendable.SendableRegistry.CallbackData;

/** Add your docs here. */
@SuppressWarnings("unused")
public class Utilities {
  static TaggedLogger logger = LoggingMaster.getLogger(Utilities.class);

  /**
   * This method makes sure the angle difference calculated falls between -180
   * degrees and 180 degrees
   * 
   * @param angle angle to be normalized
   * @return normalized angle (-180..180 degrees)
   */
  public static Angle normalizeAngle(Angle angle) {
    double radians = angle.in(Radians);
    radians = MathUtil.angleModulus(radians);
    return changeValuesUnitsTo(Radians.of(radians), angle.unit());
  }

  public static void dumpSendables(String label, String subsystemName) {
    logger.info("Dumping Sendables: {}", label);
    SendableRegistry.foreachLiveWindow(0, new Callback(subsystemName));
  }

  static class Callback implements Consumer<SendableRegistry.CallbackData> {
    String subsystemName;

    Callback(String s) {
      this.subsystemName = s;
    }

    @Override
    public void accept(CallbackData t) {
      if (subsystemName == null || subsystemName.equals(t.subsystem)) {
        logger.info("- {} -> {}", t.subsystem, t.name);
      }
    }
  }

  public static class SlidingWindowStats {
    LinkedList<Double> values = new LinkedList<>();

    int maxSize;

    int size;

    double mean;
    double stdDev;
    int flyers;

    boolean dirty = true;

    public SlidingWindowStats(int maxSize) {
      this.maxSize = maxSize;
      clear();
    }

    public void clear() {
      values.clear();
      size = 0;
      mean = 0.0;
      stdDev = 0.0;
      flyers = 0;
    }

    public void addValue(double v) {
      if (size == maxSize) {
        values.removeFirst();
        size--;
      }
      values.addLast(v);
      size++;

      dirty = true;
    }

    void updateStats() {
      mean = sum(values) / values.size();

      double sumOfSquaredDifferences = 0;
      for (var v : values) {
        sumOfSquaredDifferences += (v - mean) * (v - mean);
      }
      stdDev = sumOfSquaredDifferences / values.size();

      flyers = 0;
      for (var v : values) {
        if (Math.abs(v - mean) > (3 * stdDev)) {
          flyers++;
        }
      }

      dirty = false;
    }

    public int getSize() {
      if (dirty)
        updateStats();
      return size;
    }

    public double getMean() {
      if (dirty)
        updateStats();
      return mean;
    }

    public double getStdDev() {
      if (dirty)
        updateStats();
      return stdDev;
    }

    public int getFlyers() {
      if (dirty)
        updateStats();
      return flyers;
    }

    @Override
    public String toString() {
      if (dirty)
        updateStats();
      return "SlidingWindowStats [size=" + size + ", mean=" + mean
          + ", stdDev=" + stdDev + ", flyers=" + flyers + "]";
    }

  }

  public static double sum(List<Double> l) {
    double rv = 0;
    for (double v : l)
      rv += v;
    return rv;
  }

  public static void addDataLogForNT(String prefix) {
    String s = "/" + removeLeadingAndTrailingSlashes(prefix);
    int handle = NetworkTableInstance.getDefault().startEntryDataLog(DataLogManager.getLog(), s, s);
  }

  public static String removeLeadingAndTrailingSlashes(String s) {
    String rv = s.replaceFirst("/+$", "");
    rv = rv.replaceFirst("^/+", "");
    return rv;
  }

  public static void logMetadataToDataLog() {
    DataLog l = DataLogManager.getLog();
    String s;

    s = GitNess.getBuildTime();
    if (s != null)
      logMetadataToDataLog(l, "BuildDate", s);

    s = GitNess.getBuildHost();
    if (s != null)
      logMetadataToDataLog(l, "BuildHost", s);

    s = GitNess.getBranch(null);
    if (s != null)
      logMetadataToDataLog(l, "GitBranch", s);

    s = GitNess.getCommitDate();
    if (s != null)
      logMetadataToDataLog(l, "GitDate", s);

    Boolean dirty = GitNess.getDirty();
    if (dirty != null)
      logMetadataToDataLog(l, "GitDirty", dirty ? "Uncommitted changes" : "All changes committed");

    s = GitNess.getCommitId();
    if (s != null)
      logMetadataToDataLog(l, "GitSHA", s);

    s = GitNess.getProject(null);
    if (s != null)
      logMetadataToDataLog(l, "ProjectName", s);

    RuntimeType rt = RobotBase.getRuntimeType();
    if (s != null)
      logMetadataToDataLog(l, "RuntimeType", rt.name());

    s = RobotController.getSerialNumber();
    if (s != null && !s.equals(""))
      logMetadataToDataLog(l, "SerialNumber", s);
  }

  public static void logMetadataToDataLog(String name, String value) {
    new StringLogEntry(DataLogManager.getLog(), "/Metadata/" + name).append(value);
  }

  static void logMetadataToDataLog(DataLog l, String name, String value) {
    new StringLogEntry(l, "/Metadata/" + name).append(value);
  }

  /*
   * DW: I should be able to figure out how to do both Distance and Angle with 
   * one method using generics, but I ain't that smart. This works!
   * 
   */

  public static Distance clamp(Distance v, Distance min, Distance max) {
    if (v.lt(min)) {
      return changeValuesUnitsTo(min, v.unit());
    } else if (v.gt(max)) {
      return changeValuesUnitsTo(max, v.unit());
    }
    return v;
  }

  public static Distance changeValuesUnitsTo(Distance v, DistanceUnit u) {
    if (v.unit() == u) return v;
    return u.of(v.in(u));
  }

  public static Angle clamp(Angle v, Angle min, Angle max) {
    if (v.lt(min)) {
      return changeValuesUnitsTo(min, v.unit());
    } else if (v.gt(max)) {
      return changeValuesUnitsTo(max, v.unit());
    }
    return v;
  }

  private static Angle changeValuesUnitsTo(Angle v, AngleUnit u) {
    if (v.unit() == u) return v;
    return u.of(v.in(u));
  }
}
