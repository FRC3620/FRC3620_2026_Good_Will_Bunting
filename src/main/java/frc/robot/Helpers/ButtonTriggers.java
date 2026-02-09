package frc.robot.Helpers;

import org.usfirst.frc3620.odo.OdoIdsFlySky;
import org.usfirst.frc3620.odo.OdoIdsXBox;
import org.usfirst.frc3620.odo.OdoJoystick;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ButtonTriggers {

    public final Trigger SWAOn;
    public final Trigger SWAOff;


    public ButtonTriggers(OdoJoystick joystick) {


        SWAOn = joystick.button(OdoIdsFlySky.ButtonId.SWA, OdoIdsXBox.ButtonId.A);
        SWAOff = SWAOn.negate();

        


    }
}
