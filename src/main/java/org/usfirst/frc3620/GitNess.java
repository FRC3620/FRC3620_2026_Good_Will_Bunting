package org.usfirst.frc3620;

import java.io.File;
import java.io.FileInputStream;
import java.io.InputStream;
import java.util.Properties;

import edu.wpi.first.wpilibj.Filesystem;

public class GitNess {
  private static Properties instance;

  static {
    instance = new Properties();
    File propFile = new File(Filesystem.getDeployDirectory(), "git.properties");

    // .close of resourceStream automagically happens
    try (InputStream resourceStream = new FileInputStream(propFile)) {
      instance.load(resourceStream);
    } catch (Exception ex) {
      instance.setProperty("oopsie", ex.toString());
    }
  }

  public static Properties gitProperties() {
    return instance;
  }

  public static String getBranch(String defaultValue) {
    return instance.getProperty("git.branch", defaultValue);
  }

  public static String getBuildTime() {
    return instance.getProperty("build.time");
  }

  public static String getBuildHost() {
    return instance.getProperty("git.build.host");
  }

  public static String getCommitId() {
    return instance.getProperty("git.commit.id");
  }

  public static String getCommitDate() {
    return instance.getProperty("git.commit.time");
  }

  public static String getDescription(String defaultValue) {
    return instance.getProperty("git.commit.id.describe", defaultValue);
  }

  public static Boolean getDirty() {
    String s = instance.getProperty("git.dirty");
    if (s == null)
      return null;
    return Boolean.parseBoolean(s);
  }

  public static String getProject(String defaultValue) {
    return instance.getProperty("project.dir", defaultValue);
  }

  public static String gitDescription() {
    StringBuilder sb = new StringBuilder();
    String s;
    
    sb.append(getProject("unknown-project"));
    sb.append(" ");

    sb.append(getCommitId());
    Boolean dirty = getDirty();
    if (dirty == null) {
        sb.append("-unknownDirtyness");
    } else {
        sb.append (dirty ? "-dirty" : "");
    }

    sb.append(" Branch=");
    sb.append(getBranch("(null)"));

    s = getBuildTime();
    if (s != null) {
        sb.append(" Build-time=\"");
        sb.append(s);
        sb.append( "\"");
    }

    s = getBuildHost();
    if (s != null) {
        sb.append(" Build-host=\"");
        sb.append(s);
        sb.append( "\"");
    }

    return sb.toString();
  }

  public static String gitString() {
    return gitProperties().toString();
  }

}
