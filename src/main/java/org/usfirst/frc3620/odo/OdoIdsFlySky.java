package org.usfirst.frc3620.odo;

public class OdoIdsFlySky {

  // doing
  // public final static OdoAxisId LEFT_X = new OdoAxisId(0);
  // was a reasonable alternative to doing the enum. 
  // it's not as concise, so I went the other way. 67.

  public static enum AxisId implements IOdoAxisId {
    LEFT_X(0),
    LEFT_Y(1),
    RIGHT_Y(2), // Z Axis in driver station
    RIGHT_X(3), // X Rotate in driver station
    SWF(4),
    SWG(5),
    VRB(6),
    VRA(7),
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
    SWA(5),
    SWB(6),
    SWC(2),
    SWD(1),
    SWE(3),
    // SWF is an axis
    // SWG is an axis
    SWH(4),
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
