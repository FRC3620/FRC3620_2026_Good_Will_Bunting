package frc.robot.Helpers;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class HubTracker {
    private static String currentGameData = "";

    public static boolean isAllianceHubActive() {
        Alliance activeAlliance = getActiveAlliance();
        Alliance teamAlliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        if (activeAlliance == null) {
            return false;
        }
        return activeAlliance == teamAlliance;
    }

    public static Alliance getActiveAlliance(double currentTime) {
        if (DriverStation.isAutonomous()) {
            return DriverStation.getAlliance().orElse(Alliance.Blue);
        }

        if (currentGameData.length() == 0) {
            currentGameData = DriverStation.getGameSpecificMessage();
            if (currentGameData.length() == 0) {
                return DriverStation.getAlliance().orElse(Alliance.Blue);
            }
        }

        // 'R' = Red goes inactive FIRST, so Red is active in shifts 2 & 4
        // 'B' = Blue goes inactive FIRST, so Blue is active in shifts 2 & 4
        Alliance inactiveFirst = currentGameData.charAt(0) == 'R' ? Alliance.Red : Alliance.Blue;
        Alliance shift1Active = inactiveFirst == Alliance.Red ? Alliance.Blue : Alliance.Red;

        if (currentTime > 130 || currentTime <= 30) {
            // Transition / End game — both alliances active, return own alliance
            return DriverStation.getAlliance().orElse(Alliance.Blue);
        } else if (currentTime > 105) {
            return shift1Active;       // Shift 1
        } else if (currentTime > 80) {
            return inactiveFirst;      // Shift 2
        } else if (currentTime > 55) {
            return shift1Active;       // Shift 3
        } else {
            return inactiveFirst;      // Shift 4 (30 < t <= 55)
        }
    }

    public static boolean willAllianceBecomeActiveSoon() {
        double currentTime = DriverStation.getMatchTime();
        Alliance teamAlliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        Alliance currentActive = getActiveAlliance();
        Alliance futureActive = getActiveAlliance(currentTime - 10);
        return currentActive != teamAlliance && futureActive == teamAlliance;
    }

    public static Alliance getActiveAlliance() {
        return getActiveAlliance(DriverStation.getMatchTime());
    }
}