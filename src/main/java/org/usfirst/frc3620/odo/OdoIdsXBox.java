package org.usfirst.frc3620.odo;

public class OdoIdsXBox {

  public static enum AxisId implements IOdoAxisId {
    LEFT_X(0),
    LEFT_Y(1),
    LEFT_TRIGGER(2),
    RIGHT_TRIGGER(3),
    RIGHT_X(4),
    RIGHT_Y(5),
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
    A(1),
    B(2),
    X(3),
    Y(4),
    LEFT_BUMPER(5),
    RIGHT_BUMPER(6),
    BACK(7),
    START(8),
    LEFT_STICK(9),
    RIGHT_STICK(10),
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
