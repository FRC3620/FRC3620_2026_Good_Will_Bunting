package org.usfirst.frc3620;

import java.util.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.struct.Struct;

/** Add your docs here. */
public class NTStructs {
  @SuppressWarnings("rawtypes")
  static Map<String, StructPublisher> structPublishers = new TreeMap<>();

  @SuppressWarnings("rawtypes")
  static Map<String, StructArrayPublisher> structArrayPublishers = new TreeMap<>();

  static NetworkTableInstance nt = NetworkTableInstance.getDefault();

  @SuppressWarnings({ "unchecked", "rawtypes" })
  public static void publish(String topic, Object o, Struct s) {
    StructPublisher publisher = structPublishers.computeIfAbsent(topic, (unused_lambda_argument) -> nt.getStructTopic(topic, s).publish());
    if (publisher != null) {
      publisher.set(o);
    }
  }

  @SuppressWarnings({ "unchecked", "rawtypes" })
  public static void publish(String topic, Object[] o, Struct s) {
    StructArrayPublisher publisher = structArrayPublishers.computeIfAbsent(topic, (unused_lambda_argument) -> nt.getStructArrayTopic(topic, s).publish());
    if (publisher != null) {
      publisher.set(o);
    }
  }

  public static void publish(String topic, Pose2d o) {
    publish(topic, o, Pose2d.struct);
  }

  public static void publish(String topic, Pose3d o) {
    publish(topic, o, Pose3d.struct);
  }

  public static void publish(String topic, Pose3d[] o) {
    publish(topic, o, Pose3d.struct);
  }

  public static void publishToSmartDashboard(String topic, Pose2d o) {
    publish(fixupTopicForSmartDashboard(topic), o, Pose2d.struct);
  }

  public static void publishToSmartDashboard(String topic, Pose3d o) {
    publish(fixupTopicForSmartDashboard(topic), o, Pose3d.struct);
  }

  public static void publishToSmartDashboard(String topic, Pose3d[] o) {
    publish(fixupTopicForSmartDashboard(topic), o, Pose3d.struct);
  }

  static String fixupTopicForSmartDashboard (String topic) {
    return "SmartDashboard/" + topic;
  }

}
