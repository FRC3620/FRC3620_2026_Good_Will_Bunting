package frc.robot.Helpers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

import java.util.LinkedList;
import java.util.Queue;

public class RollingAveragePose3d {
  private final int windowSize;
  private final Queue<Pose3d> poses;

  private double sumX;
  private double sumY;
  private double sumZ;
  // Instead of summing the "theta" in radians,
  // sum the cos and sin of theta.
  private double sumCos;
  private double sumSin;

  public RollingAveragePose3d(int windowSize) {
    this.windowSize = windowSize;
    this.poses = new LinkedList<>();

    sumX = 0.0;
    sumY = 0.0;
    sumZ = 0.0;

    sumCos = 0.0;
    sumSin = 0.0;
  }

  public void addPose(Pose3d pose) {
    // Add the new pose
    poses.add(pose);
    sumX += pose.getX();
    sumY += pose.getY();
    sumZ += pose.getZ();

    sumCos += Math.cos(pose.getRotation().getZ());
    sumSin += Math.sin(pose.getRotation().getZ());

    // If we exceed the window size, remove the oldest pose
    if (poses.size() > windowSize) {
      Pose3d removed = poses.poll();
      sumX -= removed.getX();
      sumY -= removed.getY();
      sumZ -= removed.getZ();

      sumCos -= Math.cos(removed.getRotation().getZ());
      sumSin -= Math.sin(removed.getRotation().getZ());
    }
  }

  public Pose3d getAveragePose() {
    if (poses.isEmpty()) {
      return new Pose3d(); // Default to zero pose
    }
    int size = poses.size();

    double avgX = sumX / size;
    double avgY = sumY / size;
    double avgZ = sumZ / size;

    // Compute average rotation via atan2 of the summed cos/sin.
    double avgCos = sumCos / size;
    double avgSin = sumSin / size;
    double avgTheta = Math.atan2(avgSin, avgCos);

    return new Pose3d(avgX, avgY, avgZ, new Rotation3d(0, 0, avgTheta));
  }

  public boolean isFilled() {
    return poses.size() < 5;
  }

  public void reset() {
    poses.clear();
    sumX = 0.0;
    sumY = 0.0;
    sumZ = 0.0;
    sumCos = 0.0;
    sumSin = 0.0;
  }
}