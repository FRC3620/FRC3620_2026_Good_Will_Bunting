// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.ArrayList;
import java.util.EnumSet;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.TreeMap;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Supplier;

import org.usfirst.frc3620.NTStructs;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.LimelightSubsystem.CameraData.MegaTagData;

public class LimelightSubsystem extends SubsystemBase {

  NetworkTableInstance inst = NetworkTableInstance.getDefault();

  public static AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout
    .loadField(AprilTagFields.k2026RebuiltWelded);

  public static Optional<Alliance> color;

  public String lastLoggedError;

  public enum Camera {
    LEFT("limelight-left"),
    RIGHT("limelight-right");

    public final String limelightName;

    Camera(String limelightName) {
      this.limelightName = limelightName;
    }
  }

  public class CameraData {
    final String limelightName;
    public final MegaTagData megaTag1 = new MegaTagData("megaTag1");
    public final MegaTagData megaTag2 = new MegaTagData("megaTag2");
    boolean useThisCamera = true;
    int countOfSwerveUpdatesFromThisCamera = 0;

    CameraData(Camera c) {
      limelightName = c.limelightName;
      /*
       * see
       * https://docs.wpilib.org/en/stable/docs/software/networktables/listening-for-
       * change.html#using-networktableinstance-to-listen-for-changes
       * 
       * we set the haveNew* atomic(s) when the corresponding network entry is
       * updated, and in periodic() we only go through all the work of reading
       * the network entry if it had been updated.
       */
      DoubleArrayEntry entry = LimelightHelpers.getLimelightDoubleArrayEntry(c.limelightName, "botpose_wpiblue");
      inst.addListener(
          entry,
          EnumSet.of(NetworkTableEvent.Kind.kValueAll),
          event -> megaTag1.haveNewPose.set(true));

      entry = LimelightHelpers.getLimelightDoubleArrayEntry(c.limelightName, "botpose_orb_wpiblue");
      inst.addListener(
          entry,
          EnumSet.of(NetworkTableEvent.Kind.kValueAll),
          event -> megaTag2.haveNewPose.set(true));
    }

    public String getLimelightName() {
      return limelightName;
    }

    public CameraData withUseThisCamera(boolean b) {
      useThisCamera = b;
      return this;
    }

    public boolean shouldUseThisCamera() {
      return useThisCamera;
    }

    public int bumpCountOfSwerveUpdatesFromThisCamera() {
      return ++countOfSwerveUpdatesFromThisCamera;
    }

    public class MegaTagData {
      // gets set whenever we have new data
      AtomicBoolean haveNewPose = new AtomicBoolean(false);

      final String megaTagName;

      PoseEstimate poseEstimate;

      double distanceToClosestSeenTarget;

      MegaTagData(String megaTagName) {
        this.megaTagName = megaTagName;
      }

      public String getMegaTagName() {
        return megaTagName;
      }

      public PoseEstimate getPoseEstimate() {
        return poseEstimate;
      }

      public String getLimelightName() {
        return limelightName;
      }

      public double getDistanceToClosestSeenTarget() {
        return distanceToClosestSeenTarget;
      }
    }

  }

  Map<Camera, CameraData> allCameraData = new TreeMap<>();
  Set<CameraData> allCameraDataAsSet;

  /** Creates a new LimelightSubsystem. */
  public LimelightSubsystem() {

    allCameraData.put(Camera.LEFT, new CameraData(Camera.LEFT)); // Camera Data front
    allCameraData.put(Camera.RIGHT, new CameraData(Camera.RIGHT)); // Camera Data Back
    allCameraData = Map.copyOf(allCameraData); // make immutable
    allCameraDataAsSet = Set.copyOf(allCameraData.values());

  }

  // Processes MegaTag Data
  void processMegaTag(MegaTagData megaTagData, Supplier<PoseEstimate> supplier, Pose2d currentPose) {
    if (megaTagData.haveNewPose.getAndSet(false)) {
      PoseEstimate m = supplier.get();

      if (m != null) {
        megaTagData.poseEstimate = m;

        var prefix = "frc3620/vision/" + megaTagData.getLimelightName() + "/" + megaTagData.megaTagName
            + "/";

        SmartDashboard.putNumber(prefix + "targetCount", m.tagCount);
        //NTStructs.publishToSmartDashboard(prefix + "poseEstimate", m.pose);

        /*StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault()
          .getStructTopic(prefix + "poseEstimate", Pose2d.struct).publish();
        publisher.set(m.pose);
        */
        // it doesn't seem that poses published to NT make it into the
        // wpilog file via NetworkTableInstance.startEntryDataLog, so let's be
        // explicit
        // DogLog.log(prefix + "poseEstimate", m.pose);

        if (currentPose != null) {
          SmartDashboard.putNumber(prefix + "distanceFromSwervePose",
              currentPose.getTranslation().getDistance(m.pose.getTranslation()));
        }

        if (aprilTagFieldLayout != null) {
          List<Pose3d> targetPoses = new ArrayList<>();

          double distanceToClosestSeenTarget = 100000;

          for (var fiducial : m.rawFiducials) {
            Optional<Pose3d> aprilTagPose = aprilTagFieldLayout.getTagPose(fiducial.id);
            if (aprilTagPose.isPresent()) {
              targetPoses.add(aprilTagPose.get());
              double distanceToThisTag = m.pose.getTranslation()
                  .getDistance(aprilTagPose.get().getTranslation().toTranslation2d());
              if (distanceToThisTag < distanceToClosestSeenTarget) {
                distanceToClosestSeenTarget = distanceToThisTag;
              }
              megaTagData.distanceToClosestSeenTarget = distanceToClosestSeenTarget;
              SmartDashboard.putNumber(prefix + "distance to closest seen tag", distanceToClosestSeenTarget);
            }
          }
          var targetPosesArray = targetPoses.toArray(new Pose3d[0]);
          NTStructs.publish(prefix + "targets", targetPosesArray);

          /*StructArrayPublisher<Pose3d> publisher2 = NetworkTableInstance.getDefault()
            .getStructArrayTopic(prefix + "targets", Pose3d.struct).publish();
          publisher2.set(targetPosesArray);*/

          // it doesn't seem that poses published to NT make it into the
          // wpilog file via NetworkTableInstance.startEntryDataLog, so let's be
          // explicit
          // DogLog.log(prefix + "targets", targetPosesArray);
        }
      }
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // gets alliance color
    color = DriverStation.getAlliance();

    // added this to handle case where color is not yet set, otherwise we blow up in
    // the simulator
    if (color.isEmpty())
      return;

    double yaw = 0;
    double yawRate = 0;
    double pitch = 0;
    SwerveSubsystem sd = null;
    Pose2d currentSwervePose = null;

    if (RobotContainer.swerveSubsystem != null) {
      sd = RobotContainer.swerveSubsystem;
      yaw = sd.getStateCopy().Pose.getRotation().getDegrees();
      SmartDashboard.putNumber("Vision.yaw", yaw);
      pitch = sd.getPigeon2().getPitch().getValueAsDouble();
      yawRate = sd.getPigeon2().getAngularVelocityZWorld().getValueAsDouble();
      currentSwervePose = sd.getStateCopy().Pose;
    }

    for (var cameraData : allCameraData.values()) {
      var sdPrefix = "frc3620/vision/" + cameraData.getLimelightName() + "/";

      LimelightHelpers.SetRobotOrientation(cameraData.limelightName, yaw, yawRate, pitch, 0, 0, 0);
      processMegaTag(cameraData.megaTag1, () -> LimelightHelpers.getBotPoseEstimate_wpiBlue(cameraData.limelightName),
          currentSwervePose);
      processMegaTag(cameraData.megaTag2,
          () -> LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraData.limelightName), currentSwervePose);

      // update robot odometry from vision

      String error = "";
      if (!cameraData.shouldUseThisCamera()) {
        error = "Ignoring this camera";
      }
      if (Math.abs(yawRate) > 720) {
        // if our angular velocity is greater than 720 degrees per second, ignore vision
        // updates
        error = "Angular Velocity";
      } else if (cameraData.megaTag2.poseEstimate == null) {
        error = "megaTag2Pose = null";
      } else if (cameraData.megaTag2.poseEstimate.tagCount == 0) {
        error = "No Visible Tags";
      } else if (cameraData.megaTag2.getDistanceToClosestSeenTarget() > 4.75) {
        error = "Closest Tag Too Far";
      } else if (sd == null) {
        error = "No Swerve Drive";
      } else if (cameraData.megaTag2.poseEstimate.pose.getX() >= 17) {
        error = "Pose has too much X";
      } else if (cameraData.megaTag2.poseEstimate.pose.getX() <= 0) {
        error = "Pose has too little X";
      } else if (cameraData.megaTag2.poseEstimate.pose.getY() >= 8.5) {
        error = "Pose has too much Y";
      } else if (cameraData.megaTag2.poseEstimate.pose.getY() <= 0) {
        error = "Pose has too little Y";
      }
      if (sd != null && error.length() == 0) {
        double distanceError = sd.getState().Pose.getTranslation()
            .getDistance(cameraData.megaTag2.poseEstimate.pose.getTranslation());
          double numberTargetsSeen = cameraData.megaTag2.poseEstimate.tagCount;

        double translationStdDev = Math.min(50.0, Math.max(0.4, numberTargetsSeen * distanceError * 2.0));
        // double rotationStdDev = Math.min(50.0, Math.max(0.3, distanceError * 0.3));

        // sd.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));//originally
        // .7, .7, 9999999
        sd.setVisionMeasurementStdDevs(VecBuilder.fill(translationStdDev, translationStdDev, 9999999));

        // If the QuestNav isn't connected or isn't tracking, use Limelight
        if(!RobotContainer.questNavSubsystem.getQuestNavConnected() || !RobotContainer.questNavSubsystem.getQuestNavIsTracking()){
          sd.addVisionMeasurement(cameraData.megaTag2.poseEstimate.pose, cameraData.megaTag2.poseEstimate.timestampSeconds);
        }

        NTStructs.publish(sdPrefix + "megaTag2PoseEstimate", cameraData.megaTag2.poseEstimate.pose);
        NTStructs.publish(sdPrefix + "megaTag1PoseEstimate", cameraData.megaTag1.poseEstimate.pose);


        int updateCount = cameraData.bumpCountOfSwerveUpdatesFromThisCamera();
        SmartDashboard.putNumber(sdPrefix + "swervePoseUpdates", updateCount);

      }
      if (error != lastLoggedError) {
        // log if it changed

        // and remember!
        lastLoggedError = error;
      }
      SmartDashboard.putString(sdPrefix + "rejectionMessage", error);
    }

    // SmartDashboard.putNumber("errorBetweenMegaTag2Readings", getErrorBetweenCameras().in(Meters));
  }

  public CameraData getCameraData(Camera camera) {
    return allCameraData.get(camera);
  }

  public Set<CameraData> getAllCameraData() {
    return allCameraDataAsSet;
  }

  public Rotation2d getMegaTag1Rotation() {
    List<Rotation2d> rotations = new ArrayList<>();

    for (CameraData cdata : allCameraData.values()) {
      PoseEstimate pe = cdata.megaTag1.getPoseEstimate();
      if(pe != null) {
        rotations.add(pe.pose.getRotation());
      }
    }
    
    double sinSum = 0;
    double cosSum = 0;
    for (Rotation2d rot : rotations) {
      sinSum += rot.getSin();
      cosSum += rot.getCos();
    }

    return new Rotation2d(cosSum, sinSum);
  }

  public Distance getErrorBetweenCameras() {
    List<Pose2d> pes = new ArrayList<>();
    Distance error = Meters.of(0);
    for(CameraData cdata : allCameraData.values()) {
      Pose2d pe = cdata.megaTag2.getPoseEstimate().pose;
      if (pe != null) {
        pes.add(pe);
      }
    } 
    

    for (int i = 0; i < pes.size() - 1; i++) {
      error = error.plus(Meters.of(pes.get(i).getTranslation().getDistance(pes.get(i+1).getTranslation())));
    }

    return error;
  }
  
}
