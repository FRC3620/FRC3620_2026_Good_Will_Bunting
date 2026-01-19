package org.usfirst.frc3620;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.*;

@SuppressWarnings("unused")
public class JoystickAnalogButton extends Trigger {
	public JoystickAnalogButton(GenericHID joystick, int axis) {
		this(joystick, axis, 0.1);
	}

	public JoystickAnalogButton(GenericHID joystick, int axis, double threshold) {
		super(new JoystickAnalogButtonBooleanSupplier(joystick, axis, threshold));
	}

	static class JoystickAnalogButtonBooleanSupplier implements BooleanSupplier {
		GenericHID stick;
		int axis;
		double threshold;

		public JoystickAnalogButtonBooleanSupplier (GenericHID joystick, int axis, double threshold) {
			this.stick = joystick;
			this.axis = axis;
			this.threshold = threshold;
		}

		public boolean getAsBoolean() {
			if (threshold > 0.0) {
				return stick.getRawAxis(axis) > threshold;
			} else {
				return stick.getRawAxis(axis) < threshold;
			}
		}
	}
}