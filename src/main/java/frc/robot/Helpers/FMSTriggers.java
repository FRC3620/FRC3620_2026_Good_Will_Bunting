package frc.robot.Helpers;

import java.util.Optional;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class FMSTriggers {

    public final Trigger isActivePeriod;
    public final Trigger startOfInactivePeriod;
    public final Trigger isEndgame;
    public final Trigger aboutToBecomeActive;
 //   public final Trigger almostActivePeriod;
    //public final Trigger fmsTriggersOff;

    public FMSTriggers() {


        //SmartDashboard.putBoolean("triggers On", true);

        isActivePeriod = new Trigger(() -> HubTracker.isAllianceHubActive());
        aboutToBecomeActive =
        new Trigger(() -> HubTracker.willAllianceBecomeActiveSoon());
        startOfInactivePeriod = aboutToBecomeActive.negate().and(isActivePeriod.negate());
        isEndgame = new Trigger(() -> DriverStation.getMatchTime() <= 20);
}
      

       // fmsTriggersOff = new Trigger(() -> !useFMSTriggers.getAsBoolean());

    }


