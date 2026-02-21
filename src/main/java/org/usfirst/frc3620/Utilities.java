// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.usfirst.frc3620;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import java.util.*;
import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.LinkedList;
import java.util.List;
import java.util.function.Consumer;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.util.regex.PatternSyntaxException;

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
    if (v.unit() == u)
      return v;
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
    if (v.unit() == u)
      return v;
    return u.of(v.in(u));
  }

  // Source - https://stackoverflow.com/a/1248627
  // Posted by Dave Ray, modified by community. See post 'Timeline' for change
  // history
  // Retrieved 2026-02-15, License - CC BY-SA 2.5

  public static String convertGlobToRegEx(String line) {
    line = line.trim();
    int strLen = line.length();
    StringBuilder sb = new StringBuilder(strLen);
    // Remove beginning and ending * globs because they're useless
    if (line.startsWith("*")) {
      line = line.substring(1);
      strLen--;
    }
    if (line.endsWith("*")) {
      line = line.substring(0, strLen - 1);
      strLen--;
    }
    boolean escaping = false;
    int inCurlies = 0;
    for (char currentChar : line.toCharArray()) {
      switch (currentChar) {
        case '*':
          if (escaping)
            sb.append("\\*");
          else
            sb.append(".*");
          escaping = false;
          break;
        case '?':
          if (escaping)
            sb.append("\\?");
          else
            sb.append('.');
          escaping = false;
          break;
        case '.':
        case '(':
        case ')':
        case '+':
        case '|':
        case '^':
        case '$':
        case '@':
        case '%':
          sb.append('\\');
          sb.append(currentChar);
          escaping = false;
          break;
        case '\\':
          if (escaping) {
            sb.append("\\\\");
            escaping = false;
          } else
            escaping = true;
          break;
        case '{':
          if (escaping) {
            sb.append("\\{");
          } else {
            sb.append('(');
            inCurlies++;
          }
          escaping = false;
          break;
        case '}':
          if (inCurlies > 0 && !escaping) {
            sb.append(')');
            inCurlies--;
          } else if (escaping)
            sb.append("\\}");
          else
            sb.append("}");
          escaping = false;
          break;
        case ',':
          if (inCurlies > 0 && !escaping) {
            sb.append('|');
          } else if (escaping)
            sb.append("\\,");
          else
            sb.append(",");
          break;
        default:
          escaping = false;
          sb.append(currentChar);
      }
    }
    return sb.toString();
  }

  public static class GlobMatcher {
    List<Pattern> patterns = new ArrayList<>();

    public GlobMatcher() {

    }

    public GlobMatcher(Collection<String> globs) {
      addGlobs(globs);
    }

    public void addGlobs(Collection<String> globs) {
      for (var glob : globs) {
        addGlob(glob);
      }
    }

    public void addGlob(String glob) {
      String regexp = Utilities.convertGlobToRegEx(glob);
      Pattern pattern = null;
      try {
        pattern = Pattern.compile(regexp, Pattern.CASE_INSENSITIVE);
        logger.info("glob '{}' -> regexp '{}'", glob, pattern);
      } catch (PatternSyntaxException ex) {
        logger.warn("Unable to parse regexp from glob: '{}' -> '{}': {}", glob, regexp,
            ex);
      }
      if (pattern != null) {
        patterns.add(pattern);
      }
    }

    public boolean matches(String string) {
      for (var pattern : patterns) {
        Matcher m = pattern.matcher(string);
        if (m.find())
          return true;
      }
      return false;
    }
  }

  public static <T> T extractPrivateField(Class<T> returnClazz, Class<?> clazz, Object o, String name) {
    try {
      Field privateField = clazz.getDeclaredField(name);
      privateField.setAccessible(true);
      T rv = returnClazz.cast(privateField.get(o));
      return rv;
    } catch (SecurityException | IllegalAccessException | ClassCastException | NoSuchFieldException e) {
      throw new RuntimeException(e);
    }
  }

  public static <T> T callMethod(Class<T> returnClazz, Class<?> clazz, Object o, String name) {
    try {
      Method method = clazz.getMethod(name, new Class[] {});
      T rv = returnClazz.cast(method.invoke(o, new Object[] {}));
      return rv;
    } catch (SecurityException | IllegalAccessException | NoSuchMethodException | InvocationTargetException e) {
      throw new RuntimeException(e);
    }
  }

}
