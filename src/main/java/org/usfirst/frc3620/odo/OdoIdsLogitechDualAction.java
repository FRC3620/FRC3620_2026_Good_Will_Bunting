package org.usfirst.frc3620.odo;

public class OdoIdsLogitechDualAction {
  public static String name = "Logitech Dual Action";

  /*
   * these mappings are for if the Mode light is OFF. If the Mode light is one,
   * then the POV hat becomes axes 0 and 1 and the left joystick controls POV
   */

  public static enum AxisId implements IOdoAxisId {
    LEFT_X(0),
    LEFT_Y(1),
    RIGHT_X(2), // Z Axis in driver station
    RIGHT_Y(3), // X Rotate in driver station
    ;

    int axisNumber;

    AxisId(int axisNumber) {
      this.axisNumber = axisNumber;
    }

    @Override
    public int getAxisNumber() {
      return axisNumber;
    }

  }

  public static enum ButtonId implements IOdoButtonId {
    B1(1),
    B2(2),
    B3(3),
    B4(4),
    B5(5),
    B6(6),
    B7(7),
    B8(8),
    B9(9),
    B10(10),
    LEFT_STICK_PRESS(11),
    RIGHT_STICK_PRESS(12),
    ;

    int buttonNumber;

    ButtonId(int buttonNumber) {
      this.buttonNumber = buttonNumber;
    }

    @Override
    public int getButtonNumber() {
      return buttonNumber;
    }
  }
}
