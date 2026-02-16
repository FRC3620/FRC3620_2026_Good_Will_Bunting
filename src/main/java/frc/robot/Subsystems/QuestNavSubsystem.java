package frc.robot.Subsystems;

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.HealthSubsystem.HealthOptions;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

public class QuestNavSubsystem extends SubsystemBase {

  public QuestNav questNav = new QuestNav();
  private final double QUEST_NAV_HEIGHT = 23.5;
  private final double QUEST_NAV_FORWARD_CENTER_OFFSET = -11.25;
  private final double QUEST_NAV_DEGREE_YAW_OFFSET = 180;

  // private Transform2d QUEST_TO_ROBOT2D = new
  // Transform2d(Units.inchesToMeters(15.0), Units.inchesToMeters(0), new
  // Rotation2d(0));
  private Transform3d QUEST_TO_ROBOT = new Transform3d(Units.inchesToMeters(QUEST_NAV_FORWARD_CENTER_OFFSET), 0,
      Units.inchesToMeters(QUEST_NAV_HEIGHT),
      new Rotation3d(Units.degreesToRadians(0), 0, Units.degreesToRadians(QUEST_NAV_DEGREE_YAW_OFFSET)));
  private SwerveSubsystem swerveSubsystem;
  Pose3d roboPose = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));

  // Define the publisher as a class-level variable to keep it active
  StructPublisher<Pose3d> posePub = NetworkTableInstance.getDefault()
      .getStructTopic("QuestNavPose3d", Pose3d.struct)
      .publish();

  /** Creates a new QuestNav. */
  public QuestNavSubsystem(SwerveSubsystem swerveSubsystem,
      Pose3d initialQuestNavPose) {

    RobotContainer.healthSubsystem.addHealthyBooleanSupplier(() -> getQuestNavConnected(), "is Questnav Connected",
        new HealthOptions().withShowAlertWhenBad(true));
    RobotContainer.healthSubsystem.addHealthyBooleanSupplier(() -> getQuestNavIsTracking(), "is Questnav Tracking",
        new HealthOptions().withShowAlertWhenBad(true));
    RobotContainer.healthSubsystem.addHealthyBooleanSupplier(() -> isQuestnavSufficientlyCharged(), "is Questnav Sufficiently Charged",
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
        Pose3d robotPose = questPose.transformBy(QUEST_TO_ROBOT.inverse());

        // Add the mesaurement to the pose Estimator
        if (swerveSubsystem != null) {
          swerveSubsystem.addVisionMeasurement(robotPose.toPose2d(), timestamp, QUESTNAV_STD_DEVS);
        }
        roboPose = robotPose;

      }
    }
  }

  public boolean isQuestnavSufficientlyCharged() {
    if (getQuestNavPower() < 15)
      return false;
    else {
      return true;
    }
  }

  public void setQuestNavPose(Pose3d pose) {
    questNav.setPose(pose.transformBy(QUEST_TO_ROBOT));
  }

  public void setQuestNavPose(Pose2d pose) {
    Pose3d pose3d = new Pose3d(pose.getX(), pose.getY(), 0, new Rotation3d(pose.getRotation()));
    questNav.setPose(pose3d.transformBy(QUEST_TO_ROBOT));
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

  public void zeroQuestNavPose() {
    setQuestNavPose(new Pose3d(Units.inchesToMeters(343), Units.inchesToMeters(14.5), 0,
        new Rotation3d(Units.degreesToRadians(0), 0, 0)));
  }

  public Command zeroQuestNavPoseCommand() {
    return runOnce(() -> zeroQuestNavPose());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    questNav.commandPeriodic();
    updateVisionMeasurement();

    posePub.set(roboPose);

  }
}