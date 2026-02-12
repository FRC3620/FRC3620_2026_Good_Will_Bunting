import org.junit.Test;

import frc.robot.Subsystems.HealthSubsystem.Health;

public class HealthOptionsTest {

  // .\gradlew test --tests=Health*
  @Test
  public void test00() {
    for (var h1 : Health.values()) {
      for (var h2 : Health.values()) {
        Health r = h1.worstOf(h2);
        System.out.println(h1 + " " + h2 + " -> " + r);
      }
    }

  }
}
