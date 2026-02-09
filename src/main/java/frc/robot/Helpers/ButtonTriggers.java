package frc.robot.Helpers;

import java.time.chrono.ThaiBuddhistEra;

import org.usfirst.frc3620.ChameleonController;
import org.usfirst.frc3620.FlySkyConstants;
import org.usfirst.frc3620.JoystickAnalogButton;
import org.usfirst.frc3620.XBoxConstants;
import org.usfirst.frc3620.ChameleonController.ControllerType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ButtonTriggers {

    public final Trigger SWAOn;
    public final Trigger SWAOff;


    public ButtonTriggers(ChameleonController joystick) {


        SWAOn = new Trigger(joystick.button(FlySkyConstants.BUTTON_SWA, XBoxConstants.BUTTON_A));
        SWAOff = SWAOn.negate();

        


    }
}
