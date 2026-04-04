package frc.robot.Helpers;

import edu.wpi.first.math.geometry.Translation3d;
import java.util.LinkedList;
import java.util.Queue;

public class RollingAverageVelocity {
  private final int windowSize;
  private final Queue<Translation3d> velocities;

  private double sumX;
  private double sumY;
  private double sumZ;

  public RollingAverageVelocity(int windowSize) {
    this.windowSize = windowSize;
    this.velocities = new LinkedList<>();

    sumX = 0.0;
    sumY = 0.0;
    sumZ = 0.0;
  }

  public void addVelocity(Translation3d velocity) {
    velocities.add(velocity);

    sumX += velocity.getX();
    sumY += velocity.getY();
    sumZ += velocity.getZ();

    // Remove oldest if over window
    if (velocities.size() > windowSize) {
      Translation3d removed = velocities.poll();
      sumX -= removed.getX();
      sumY -= removed.getY();
      sumZ -= removed.getZ();
    }
  }

  public Translation3d getAverageVelocity() {
    if (velocities.isEmpty()) {
      return new Translation3d();
    }

    int size = velocities.size();

    return new Translation3d(
        sumX / size,
        sumY / size,
        sumZ / size);
  }

  public boolean isFilled() {
    return velocities.size() >= windowSize;
  }

  public void reset() {
    velocities.clear();
    sumX = 0.0;
    sumY = 0.0;
    sumZ = 0.0;
  }
}