package frc.robot.Helpers;

import java.time.chrono.ThaiBuddhistEra;

import org.usfirst.frc3620.ChameleonController;
import org.usfirst.frc3620.FlySkyConstants;
import org.usfirst.frc3620.JoystickAnalogButton;
import org.usfirst.frc3620.XBoxConstants;
import org.usfirst.frc3620.ChameleonController.ControllerType;
import org.usfirst.frc3620.odo.OdoIdsFlySky;
import org.usfirst.frc3620.odo.OdoIdsXBox;
import org.usfirst.frc3620.odo.OdoJoystick;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ButtonTriggers {

    public final Trigger SWAOn;
    public final Trigger SWAOff;


    public ButtonTriggers(OdoJoystick joystick) {


        SWAOn = joystick.button(OdoIdsFlySky.ButtonId.SWA, OdoIdsXBox.ButtonId.A);
        SWAOff = SWAOn.negate();

        


    }
}
