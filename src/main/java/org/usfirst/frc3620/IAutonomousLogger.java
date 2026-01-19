package org.usfirst.frc3620;

/**
 * this is used for debugging autonomous driving. Look in FRC3620_2020_GalacticSenate
 * for an example of use: look for commit cc5a2c55d016895a3fb4bcec43eab37532ad1e86,
 * and look at SlalomCommandGroup and Slalom27 classes.
 */
@SuppressWarnings("unused")
public interface IAutonomousLogger {
  void setInitialDrivePositions(double lf, double rf, double lr, double rr);

  void setCurrentDrivePositions(double lf, double rf, double lr, double rr);

  void setLegName(String s);

  void setElapsed(double d);

  void doLog();
}
