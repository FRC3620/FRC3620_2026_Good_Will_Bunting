package org.usfirst.frc3620.logger;

import java.io.File;
import java.text.SimpleDateFormat;
import java.util.*;

import edu.wpi.first.wpilibj.RobotBase;
import org.tinylog.Logger;
import org.tinylog.TaggedLogger;

import edu.wpi.first.wpilibj.RobotController;

public class LoggingMaster {
  private static Date _timestamp = null;

  private static File _logDirectory = null;

  static {
    TimeZone.setDefault(TimeZone.getTimeZone("UTC"));
    if (RobotBase.isReal()) {
      if (_logDirectory == null)
        _logDirectory = searchForLogDirectory(new File("/u"));
      if (_logDirectory == null) {
        _logDirectory = new File("/home/lvuser/logs");
      }
    } else {
      _logDirectory = new File("./logs");
    }
    if (!_logDirectory.exists()) {
      _logDirectory.mkdir();
    }
    String logMessage = String.format("[LoggingMaster] Log directory is %s\n",
            _logDirectory.getAbsolutePath());
    System.out.print(logMessage); // NOPMD
  }

  // http://javarevisited.blogspot.com/2014/05/double-checked-locking-on-singleton-in-java.html
  public static Date getTimestamp() {
    if (_timestamp == null) { // do a quick check (no overhead from
                              // synchonized)
      synchronized (LoggingMaster.class) {
        if (_timestamp == null) { // Double checked
          if (RobotController.isSystemTimeValid()) {
            _timestamp = new Date();
            String logMessage = String.format(
                "[LoggingMaster] timestamp for logs is %s\n", convertTimestampToString(_timestamp));
            System.out.println(logMessage);
          }
        }
      }
    }
    return _timestamp;
  }

  static File searchForLogDirectory(File root) {
    // does the root directory exist?
    if (!root.isDirectory())
      return null;

    File logDirectory = new File(root, "logs");
    if (logDirectory.exists()) {
      if (!logDirectory.isDirectory())
        return null;
      if (!logDirectory.canWrite())
        return null;
    }

    return logDirectory;
  }

  public static String convertTimestampToString(Date ts) {
    SimpleDateFormat formatName = new SimpleDateFormat("yyyyMMdd-HHmmss");
    return formatName.format(ts);
  }

  public static File getLoggingDirectory() {
    return _logDirectory;
  }

  public static TaggedLogger getLogger(Class<?> clazz) {
    String s = shortenClassName(clazz.getName());
    return Logger.tag(s);
  }

  public static TaggedLogger getLogger(String loggerName) {
    return Logger.tag(loggerName);
  }

  public static String shortenClassName(String className) {
    List<String> parts = new ArrayList<>(Arrays.asList(className.split("\\.")));
    if (parts.size() == 0) return "";
    StringBuffer rv = new StringBuffer();
    while (parts.size() > 1) {
      String part = parts.remove(0);
      if (part.length() > 0) {
        rv.append(part.charAt(0));
        rv.append(".");
      }
    }
    rv.append(parts.get(0));
    return rv.toString();
  }
}