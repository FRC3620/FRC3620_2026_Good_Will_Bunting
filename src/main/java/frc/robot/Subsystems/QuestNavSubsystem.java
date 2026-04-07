package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import static edu.wpi.first.units.Units.Microseconds;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.StructLogEntry;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.util.datalog.StructLogEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Helpers.RollingAveragePose3d;
import frc.robot.Helpers.RollingAverageVelocity;
import frc.robot.Subsystems.HealthSubsystem.HealthOptions;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;
import edu.wpi.first.wpilibj.DataLogManager;
import gg.questnav.questnav.protos.wpilib.FrameDataProto;

public class QuestNavSubsystem extends SubsystemBase {

  public QuestNav questNav = new QuestNav();
  private final double QUEST_NAV_HEIGHT_CHUD = 23.5;
  private final double QUEST_NAV_FORWARD_CENTER_OFFSET_CHUD = -11.25;
  private final double QUEST_NAV_DEGREE_YAW_OFFSET_CHUD = 180;

  private final double QUEST_NAV_HEIGHT_RAPTOR = 15.5;
  private final double QUEST_NAV_FORWARD_CENTER_OFFSET_RAPTOR = -11.25 + 2.4;
  private final double QUEST_NAV_DEGREE_YAW_OFFSET_RAPTOR = 180;
  private final double QUEST_NAV_LEFT_OFFSET_RAPTOR = -6.45;

  private final RollingAveragePose3d rollingAvgPose = new RollingAveragePose3d(5);
  private final RollingAverageVelocity rollingAvgVelo = new RollingAverageVelocity(3);

  private Pose3d lastPose = null;
  private double lastTimestamp = -1;

  private LinearVelocity vx = MetersPerSecond.of(0.0);
  private LinearVelocity vy = MetersPerSecond.of(0.0);
  private AngularVelocity omega = RotationsPerSecond.of(0.0);

  private Transform3d QUEST_TO_ROBOT_CHUD = new Transform3d(Units.inchesToMeters(QUEST_NAV_FORWARD_CENTER_OFFSET_CHUD),
      0,
      Units.inchesToMeters(QUEST_NAV_HEIGHT_CHUD),
      new Rotation3d(Units.degreesToRadians(0), 0, Units.degreesToRadians(QUEST_NAV_DEGREE_YAW_OFFSET_CHUD)));

  private Transform3d QUEST_TO_ROBOT_RAPTOR = new Transform3d(
      Units.inchesToMeters(QUEST_NAV_FORWARD_CENTER_OFFSET_RAPTOR), Units.inchesToMeters(QUEST_NAV_LEFT_OFFSET_RAPTOR),
      Units.inchesToMeters(QUEST_NAV_HEIGHT_RAPTOR),
      new Rotation3d(Units.degreesToRadians(0), 0, Units.degreesToRadians(QUEST_NAV_DEGREE_YAW_OFFSET_RAPTOR)));

  private SwerveSubsystem swerveSubsystem;
  Pose3d roboPose = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));

  // Define the publisher as a class-level variable to keep it active
  private StructLogEntry<Pose3d> posePub3d = StructLogEntry.create(
      DataLogManager.getLog(),
      "QuestNav/Pose3d",
      Pose3d.struct);

  private StructLogEntry<Pose2d> posePub2d = StructLogEntry.create(
      DataLogManager.getLog(),
      "QuestNav/Pose2d",
      Pose2d.struct);


  /** Creates a new QuestNav. */
  public QuestNavSubsystem(SwerveSubsystem swerveSubsystem,
      Pose3d initialQuestNavPose) {

    RobotContainer.healthSubsystem.addHealthyBooleanSupplier(() -> getQuestNavConnected(), "Questnav is not connected",
        new HealthOptions().withShowAlertWhenBad(true));
    RobotContainer.healthSubsystem.addHealthyBooleanSupplier(() -> getQuestNavIsTracking(), "Questnav is not tracking",
        new HealthOptions().withShowAlertWhenBad(true));
    RobotContainer.healthSubsystem.addHealthyBooleanSupplier(() -> isQuestnavSufficientlyCharged(),
        "Questnav is not sufficiently charged",
        new HealthOptions().withShowAlertWhenBad(true));

    this.swerveSubsystem = swerveSubsystem;

    // Set intial Position -- Right now, this assumes we're sitting in front of
    // AprilTag 10 on the red side of the field
    /*
     * questNav.setPose(new Pose3d(Units.inchesToMeters(0), //
     * Units.inchesToMeters(0),
     * Units.inchesToMeters(0),
     * new Rotation3d(Math.toRadians(180), Math.toRadians(0), Math.toRadians(0))));
     */

    questNav.setPose(initialQuestNavPose);
  }

  public void updateAverageRobotPose(Pose3d questPose) {
    rollingAvgPose.addPose(questPose);
  }

  public Pose3d getAverageRobotPose() {
    return rollingAvgPose.getAveragePose();
  }

  public Translation3d getAverageRobotVelocity() {
    return rollingAvgVelo.getAverageVelocity();
  }

  public void updateVisionMeasurement() {

    Matrix<N3, N1> QUESTNAV_STD_DEVS = VecBuilder.fill(0.02, 0.02, 0.035);

    SmartDashboard.putBoolean("QuestNav/isConnected", questNav.isConnected());
    SmartDashboard.putBoolean("QuestNav/isTracking", questNav.isTracking());
    SmartDashboard.putNumber("QuestNav/batteryPercent", getQuestNavPower());

    if (questNav.isConnected() && questNav.isTracking()) {

      PoseFrame[] questFrames = questNav.getAllUnreadPoseFrames();

      // Loop over the pose data frames nd send them to the pose estimatior

      for (PoseFrame questFrame : questFrames) {
        // Get the Pose of the Quest
        Pose3d questPose = questFrame.questPose3d();

        // get the timestamp for when the data was sent
        double timestamp = questFrame.dataTimestamp();

        // Transform by the mount pose to get the robot pose
        Pose3d robotPose = questPose.transformBy(QUEST_TO_ROBOT_RAPTOR.inverse());

        // adds to a queue array of 5 and averages the 5 values
        updateAverageRobotPose(robotPose);

        if (rollingAvgPose.isFilled()) {
          // Add the mesaurement to the pose Estimator
          if (swerveSubsystem != null) {
            swerveSubsystem.addVisionMeasurement(rollingAvgPose.getAveragePose().toPose2d(), timestamp,
                QUESTNAV_STD_DEVS);
            roboPose = rollingAvgPose.getAveragePose();
            updateVelocity(roboPose, timestamp);
          }
        } else {
          if (swerveSubsystem != null) {
            updateVelocity(robotPose, timestamp);
            swerveSubsystem.addVisionMeasurement(robotPose.toPose2d(), timestamp, QUESTNAV_STD_DEVS);
            roboPose = robotPose;
            
          }
        }
      
      }
    }
  }

  private void updateVelocity(Pose3d pose, double timestamp) {

    if (lastPose == null) {
      lastPose = pose;
      lastTimestamp = timestamp;
      return;
    }

    double dt = timestamp - lastTimestamp;

    if (dt < 0.005 || dt > 0.25) {
      lastPose = pose;
      lastTimestamp = timestamp;
      return;
    }

    // Raw velocity
    double vxVal = (pose.getX() - lastPose.getX()) / dt;
    double vyVal = (pose.getY() - lastPose.getY()) / dt;

    double maxVel = 5.5; // m/s (tune for your robot)
    vxVal = Math.max(-maxVel, Math.min(maxVel, vxVal));
    vyVal = Math.max(-maxVel, Math.min(maxVel, vyVal));

    double maxAccel = 10.0; // m/s^2 (tune this)
    double maxDelta = maxAccel * dt;

    double prevVX = vx.in(MetersPerSecond);
    double prevVY = vy.in(MetersPerSecond);

    vxVal = Math.max(prevVX - maxDelta, Math.min(prevVX + maxDelta, vxVal));
    vyVal = Math.max(prevVY - maxDelta, Math.min(prevVY + maxDelta, vyVal));

    if (Math.abs(vxVal) < 0.005)
      vxVal = 0;
    if (Math.abs(vyVal) < 0.005)
      vyVal = 0;

    // Store filtered velocity
    vx = MetersPerSecond.of(vxVal);
    vy = MetersPerSecond.of(vyVal);

    double yawNow = pose.getRotation().getZ();
    double yawPrev = lastPose.getRotation().getZ();

    double deltaYaw = yawNow - yawPrev;
    deltaYaw = Math.atan2(Math.sin(deltaYaw), Math.cos(deltaYaw));

    omega = DegreesPerSecond.of(deltaYaw / dt);

    rollingAvgVelo.addVelocity(new Translation3d(
        vxVal,
        vyVal,
        0.0));

    // Update state
    lastPose = pose;
    lastTimestamp = timestamp;
  }

  public LinearVelocity getQuestNavVX() {
    return MetersPerSecond.of(rollingAvgVelo.getAverageVelocity().getX());
  }

  public LinearVelocity getQuestNavVY() {
    return MetersPerSecond.of(rollingAvgVelo.getAverageVelocity().getY());
  }

  public AngularVelocity getQuestNavOmega() {
    return omega;
  }

  public boolean isQuestnavSufficientlyCharged() {
    if (getQuestNavPower() < 6.5)
      return false;
    else {
      return true;
    }
  }

  public void resetQuestNavPoseRollingAvg() {
    rollingAvgPose.reset();
  }

  public void resetQuestNavVeloRollingAvg() {
    rollingAvgVelo.reset();
  }

  public void setQuestNavPose(Pose3d pose) {
    questNav.setPose(pose.transformBy(QUEST_TO_ROBOT_RAPTOR));
  }

  public void setQuestNavPose(Pose2d pose) {
    Pose3d pose3d = new Pose3d(pose.getX(), pose.getY(), 0, new Rotation3d(pose.getRotation()));
    questNav.setPose(pose3d.transformBy(QUEST_TO_ROBOT_RAPTOR));
  }

  public Pose3d getNavQuestPose3d() {
    return roboPose;
  }

  public boolean getQuestNavConnected() {
    return questNav.isConnected();
  }

  public boolean getQuestNavConnectedLongTimeQuestionMark() {

    Timer timer = new Timer();

    if (questNav.isConnected()) {
      timer.reset();
      timer.start();
    } else {
      if (timer.hasElapsed(Seconds.of(5))) {
        return false;
      }
    }

    return true;
  }

  public boolean getQuestNavIsTracking() {
    return questNav.isTracking();
  }

  public int getQuestNavPower() {
    return questNav.getBatteryPercent().getAsInt();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (questNav.isConnected() && questNav.isTracking()) {
      SmartDashboard.putNumber("QuestNav/XVelocity", getQuestNavVX().in(MetersPerSecond));
      SmartDashboard.putNumber("QuestNav/YVelocity", getQuestNavVY().in(MetersPerSecond));
      
      posePub3d.append(roboPose);
      posePub2d.append(roboPose.toPose2d());
    }
    questNav.commandPeriodic();
    updateVisionMeasurement();
  }
}