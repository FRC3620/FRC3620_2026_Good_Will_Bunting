package org.usfirst.frc3620.odo;

public class OdoIdsFlySky {

  // doing
  // public final static OdoAxisId LEFT_X = new OdoAxisId(0);
  // was a reasonable alternative to doing the enum. 
  // it's not as concise, so I went the other way. 67.

  public static enum AxisId implements IOdoAxisId {
    LEFT_X(0), // Y Axis in driver station
    LEFT_Y(1),
    RIGHT_Y(2), // Z Axis in driver station
    RIGHT_X(3), // X Rotate in driver station
    SWF(5),
    SWG(6),
    VRB(7),
    // VRA(8),
    SWB(4)
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
    SWA(1),
    SWB_UP(8),
    SWB_DOWN(7),
    SWC(2),
    SWD(3),
    SWE(4),
    // SWF is an axis
    // SWG is an axis
    SWH(5),
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
