package org.usfirst.frc3620;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

@SuppressWarnings("unused")
public class ChameleonController {
    public enum ControllerType { A, B }

    Joystick joystick;

    ControllerType currentControllerType = ControllerType.A;

    public ChameleonController (Joystick joystick) {
        this.joystick = joystick;
    }

    public String getName() {
        return joystick.getName();
    }

    public void setCurrentControllerType(ControllerType controllerType) {
        this.currentControllerType = controllerType;
    }

    public ControllerType getCurrentControllerType() {
        return currentControllerType;
    }

    public double getRawAxis(int a_axis_number, int b_axis_number) {
        if (currentControllerType == ControllerType.A) {
            return joystick.getRawAxis(a_axis_number);
        } else {
            return joystick.getRawAxis(b_axis_number);
        }
    }

    public Trigger button (int a_button_index, int b_button_index) {
        return new Trigger(new ABBooleanProvider(a_button_index, b_button_index));
    }

    class ABBooleanProvider implements BooleanSupplier {
        int a_button_index, b_button_index;

        ABBooleanProvider(int a_button_index, int b_button_index) {
            this.a_button_index = a_button_index;
            this.b_button_index = b_button_index;
        }

        @Override
        public boolean getAsBoolean() {
            if (currentControllerType == ControllerType.A) {
                return joystick.getRawButton(a_button_index);
            } else {
                return joystick.getRawButton(b_button_index);
            }
        }
    }

    public Trigger analogButton (int a_index, int b_index) {
        return new Trigger(new ABAnalogButtonProvider(a_index, b_index));
    }

    class ABAnalogButtonProvider implements BooleanSupplier {
        int a_axis_number, b_axis_number;

        ABAnalogButtonProvider(int a_axis_number, int b_axis_number) {
            this.a_axis_number = a_axis_number;
            this.b_axis_number = b_axis_number;
        }

        @Override
        public boolean getAsBoolean() {
            double a = getRawAxis(a_axis_number, b_axis_number);
            return a > 0.2;
        }
    }
}

