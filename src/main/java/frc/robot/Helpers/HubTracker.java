package frc.robot.Helpers;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
        if (SmartDashboard.getString("OVERRIDE GAME DATA", "").length() == 0) {
            SmartDashboard.putString("OVERRIDE GAME DATA", "");
        }
        
        boolean overriding = false;
        Alliance inactiveFirst = DriverStation.getAlliance().orElse(Alliance.Blue);
        
        if (DriverStation.isAutonomous()) {
            return DriverStation.getAlliance().orElse(Alliance.Blue);
        }

        if (currentGameData.length() == 0) {
            currentGameData = DriverStation.getGameSpecificMessage();
            if (currentGameData.length() == 0) {
                overriding = true;
                if (SmartDashboard.getString("OVERRIDE GAME DATA", currentGameData).length() == 1) {
                    if (SmartDashboard.getString("OVERRIDE GAME DATA", currentGameData).charAt(0) == 'R') {
                        inactiveFirst = Alliance.Red;
                    } else if (SmartDashboard.getString("OVERRIDE GAME DATA", currentGameData).charAt(0) == 'B') {
                        inactiveFirst = Alliance.Blue;
                    } 
                }
            }
        }

        // 'R' = Red goes inactive FIRST, so Red is active in shifts 2 & 4
        // 'B' = Blue goes inactive FIRST, so Blue is active in shifts 2 & 4
        if (!overriding) {
            inactiveFirst = currentGameData.charAt(0) == 'R' ? Alliance.Red : Alliance.Blue;
        }
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
        Alliance futureActive = getActiveAlliance(currentTime - 1);
        return currentActive != teamAlliance && futureActive == teamAlliance;
    }

    public static Alliance getActiveAlliance() {
        return getActiveAlliance(DriverStation.getMatchTime());
    }
}