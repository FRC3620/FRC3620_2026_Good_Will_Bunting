package org.usfirst.frc3620.odo;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.tinylog.TaggedLogger;
import org.usfirst.frc3620.logger.LoggingMaster;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class OdoJoystick {
  public enum JoystickType {
    A, B
  }

  Joystick joystick;

  TaggedLogger logger = LoggingMaster.getLogger(getClass());

  JoystickType currentJoystickType = JoystickType.A;

  public OdoJoystick(Joystick joystick) {
    this.joystick = joystick;
    setJoystickType(JoystickType.A);
  }

  public void setJoystickType(JoystickType joystickType) {
    logger.info("setting joystick type to {}", joystickType);
    this.currentJoystickType = joystickType;
  }

  public JoystickType getCurrentJoystickType() {
    return currentJoystickType;
  }

  public double getRawAxis(IOdoAxisId axis_id) {
    return joystick.getRawAxis(axis_id.getAxisNumber());
  }

  public double getRawAxis(int axis_number) {
    return joystick.getRawAxis(axis_number);
  }

  public boolean getRawButton(IOdoButtonId button_id) {
    return joystick.getRawButton(button_id.getButtonNumber());
  }

  public boolean getRawButton(int button_number) {
    return joystick.getRawButton(button_number);
  }

  public Trigger button(IOdoButtonId a_button_id, IOdoButtonId b_button_id) {
    return button(() -> getRawButton(a_button_id), () -> getRawButton(b_button_id));
  }

  public Trigger button(IOdoButtonId a_button_id, BooleanSupplier b_supplier) {
    return button(() -> getRawButton(a_button_id), b_supplier);
  }

  public Trigger button(BooleanSupplier a_supplier, IOdoButtonId b_button_id) {
    return button(a_supplier, () -> getRawButton(b_button_id));
  }

  public Trigger button(BooleanSupplier a_supplier, BooleanSupplier b_supplier) {
    return new Trigger(new ABBooleanProvider(a_supplier, b_supplier));
  }

  public class ABBooleanProvider implements BooleanSupplier {
    BooleanSupplier a_supplier, b_supplier;

    ABBooleanProvider(BooleanSupplier a_supplier, BooleanSupplier b_supplier) {
      this.a_supplier = a_supplier;
      this.b_supplier = b_supplier;
    }

    @Override
    public boolean getAsBoolean() {
      if (currentJoystickType == JoystickType.A) {
        return a_supplier.getAsBoolean();
      } else {
        return b_supplier.getAsBoolean();
      }
    }
  }

  public double getAxis(IOdoAxisId a_axis_id, IOdoAxisId b_axis_id) {
    return getAxis(() -> getRawAxis(a_axis_id), () -> getRawAxis(b_axis_id));
  }

  public double getAxis(DoubleSupplier a_supplier, IOdoAxisId b_axis_id) {
    return getAxis(a_supplier, () -> getRawAxis(b_axis_id));
  }

  public double getAxis(IOdoAxisId a_axis_id, DoubleSupplier b_supplier) {
    return getAxis(() -> getRawAxis(a_axis_id), b_supplier);
  }

  public double getAxis(DoubleSupplier a_supplier, DoubleSupplier b_supplier) {
    if (currentJoystickType == JoystickType.A) {
      return a_supplier.getAsDouble();
    } else {
      return b_supplier.getAsDouble();
    }
  }

  public class ABDoubleProvider implements DoubleSupplier {
    DoubleSupplier a_supplier, b_supplier;

    ABDoubleProvider(DoubleSupplier a_supplier, DoubleSupplier b_supplier) {
      this.a_supplier = a_supplier;
      this.b_supplier = b_supplier;
    }

    @Override
    public double getAsDouble() {
      if (currentJoystickType == JoystickType.A) {
        return a_supplier.getAsDouble();
      } else {
        return b_supplier.getAsDouble();
      }
    }
  }

}