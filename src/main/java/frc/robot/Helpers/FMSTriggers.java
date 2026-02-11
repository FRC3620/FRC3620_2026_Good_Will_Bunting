package frc.robot.Helpers;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class FMSTriggers {

    public final Trigger isActivePeriod;
    public final Trigger isInactivePeriod;
    public final Trigger isEndgame;

    public FMSTriggers(Optional<Alliance> alliance) {

        isActivePeriod = new Trigger(() -> HubTracker.isAllianceHubActive());
        isInactivePeriod = isActivePeriod.negate();
        isEndgame = new Trigger(() -> DriverStation.getMatchTime() <= 20);
    }

}
