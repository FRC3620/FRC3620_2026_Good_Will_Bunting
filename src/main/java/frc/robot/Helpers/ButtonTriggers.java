package frc.robot.Helpers;

import org.usfirst.frc3620.odo.OdoIdsFlySky;
import org.usfirst.frc3620.odo.OdoIdsXBox;
import org.usfirst.frc3620.odo.OdoJoystick;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ButtonTriggers {

    public final Trigger SWAOn;
    public final Trigger SWAOff;

/* 
    public final Trigger intakeIn;
    public final Trigger intakeOut;
    public final Trigger climb;
    public final Trigger manualHoard;
    public final Trigger manualPass;
    public final Trigger deadeyeMode;
    public final Trigger exitDeadeye;
    public final Trigger manualShoot;

*/
    public ButtonTriggers(OdoJoystick joystick) {


        SWAOn = joystick.button(OdoIdsFlySky.ButtonId.SWA, OdoIdsXBox.ButtonId.A);
        SWAOff = SWAOn.negate();


        /*to be assigned later
        intakeIn = new Trigger(null);
        intakeOut = intakeIn.negate();
        climb = new Trigger(null);
        manualHoard = new Trigger(null);
        manualPass = manualHoard.negate();
        deadeyeMode = new Trigger(null);
        exitDeadeye = deadeyeMode.negate();
        manualShoot = new Trigger(null);
*/


        


    }
}
