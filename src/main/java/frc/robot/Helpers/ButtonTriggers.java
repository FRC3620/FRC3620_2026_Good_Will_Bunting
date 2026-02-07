package frc.robot.Helpers;

import org.usfirst.frc3620.ChameleonController;
import org.usfirst.frc3620.FlySkyConstants;
import org.usfirst.frc3620.XBoxConstants;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ButtonTriggers {

    public final Trigger aButtonPressed;
    public final Trigger bButtonPressed;
    public final Trigger xButtonPressed;
    public final Trigger yButtonPressed;
    
    public final Trigger rightTrigger;
    public final Trigger leftTrigger;

    public ButtonTriggers(ChameleonController joystick) {

        aButtonPressed = new Trigger(
                joystick.button(XBoxConstants.BUTTON_A, FlySkyConstants.BUTTON_SWD));

        bButtonPressed = new Trigger(
                joystick.button(XBoxConstants.BUTTON_B, FlySkyConstants.BUTTON_SWA));
        xButtonPressed = new Trigger(
                joystick.button(XBoxConstants.BUTTON_X, FlySkyConstants.BUTTON_SWF));
        
        yButtonPressed = new Trigger(
                joystick.button(XBoxConstants.BUTTON_Y, FlySkyConstants.BUTTON_SWC));

        rightTrigger = new Trigger(joystick.analogButton(XBoxConstants.AXIS_RIGHT_TRIGGER, FlySkyConstants.AXIS_SWH));
        leftTrigger = new Trigger(joystick.analogButton(XBoxConstants.AXIS_LEFT_TRIGGER, FlySkyConstants.AXIS_SWE));
    }
}
