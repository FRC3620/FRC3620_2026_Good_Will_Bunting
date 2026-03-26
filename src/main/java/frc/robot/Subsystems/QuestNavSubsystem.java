package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Helpers.RollingAveragePose3d;
import frc.robot.Subsystems.HealthSubsystem.HealthOptions;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

public class QuestNavSubsystem extends SubsystemBase {

  public QuestNav questNav = new QuestNav();
  private final double QUEST_NAV_HEIGHT_CHUD = 23.5;
  private final double QUEST_NAV_FORWARD_CENTER_OFFSET_CHUD = -11.25;
  private final double QUEST_NAV_DEGREE_YAW_OFFSET_CHUD = 180;

  private final double QUEST_NAV_HEIGHT_RAPTOR = 15.5;
  private final double QUEST_NAV_FORWARD_CENTER_OFFSET_RAPTOR = -11.25 + 2.4;
  private final double QUEST_NAV_DEGREE_YAW_OFFSET_RAPTOR = 180;
  private final double QUEST_NAV_LEFT_OFFSET_RAPTOR = -6.45;

  private final RollingAveragePose3d rollingAvg = new RollingAveragePose3d(5);

  private Pose3d lastPose = null;
  private double lastTimestamp = -1;

  private LinearVelocity vx = MetersPerSecond.of(0.0);
  private LinearVelocity vy = MetersPerSecond.of(0.0);
  private AngularVelocity omega = RotationsPerSecond.of(0.0);

  // private Transform2d QUEST_TO_ROBOT2D = new
  // Transform2d(Units.inchesToMeters(15.0), Units.inchesToMeters(0), new
  // Rotation2d(0));
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
  StructPublisher<Pose3d> posePub = NetworkTableInstance.getDefault()
      .getStructTopic("QuestNavPose3d", Pose3d.struct)
      .publish();

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
    rollingAvg.addPose(questPose);
  }

  public Pose3d getAverageRobotPose() {
    return rollingAvg.getAveragePose();
  }

  public void updateVisionMeasurement() {

    Matrix<N3, N1> QUESTNAV_STD_DEVS = VecBuilder.fill(0.02, 0.02, 0.035);

    SmartDashboard.putBoolean("QuestNav.isConnected", questNav.isConnected());
    SmartDashboard.putBoolean("QuestNav.isTracking", questNav.isTracking());
    SmartDashboard.putNumber("QuestNav.batteryPercent", getQuestNavPower());

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

        if (rollingAvg.isFilled()) {
          // Add the mesaurement to the pose Estimator
          if (swerveSubsystem != null) {
            swerveSubsystem.addVisionMeasurement(rollingAvg.getAveragePose().toPose2d(), timestamp, QUESTNAV_STD_DEVS);
            roboPose = rollingAvg.getAveragePose();
            updateVelocity(roboPose, timestamp);
          }
        } else {
          updateVelocity(robotPose, timestamp);
          swerveSubsystem.addVisionMeasurement(robotPose.toPose2d(), timestamp, QUESTNAV_STD_DEVS);
          roboPose = robotPose;
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

    if (dt <= 0)
      return;

    vx = MetersPerSecond.of((pose.getX() - lastPose.getX()) / dt);
    vy = MetersPerSecond.of((pose.getY() - lastPose.getY()) / dt);

    double yawNow = pose.getRotation().getZ();
    double yawPrev = lastPose.getRotation().getZ();

    omega = DegreesPerSecond.of((yawNow - yawPrev) / dt);

    lastPose = pose;
    lastTimestamp = timestamp;
  }

  public LinearVelocity getQuestNavVX() {
    return vx;
  }

  public LinearVelocity getQuestNavVY() {
    return vy;
  }

  public AngularVelocity getQuestNavOmega() {
    return omega;
  }

  public boolean isQuestnavSufficientlyCharged() {
    if (getQuestNavPower() < 15)
      return false;
    else {
      return true;
    }
  }

  public void resetQuestNavPoseRollingAvg() {
    rollingAvg.reset();
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
      SmartDashboard.putNumber("QuestNav.XVelocity", getQuestNavVX().in(MetersPerSecond));
      SmartDashboard.putNumber("QuestNav.YVelocity", getQuestNavVY().in(MetersPerSecond));
    }

    questNav.commandPeriodic();
    updateVisionMeasurement();

    posePub.set(roboPose);

  }
}