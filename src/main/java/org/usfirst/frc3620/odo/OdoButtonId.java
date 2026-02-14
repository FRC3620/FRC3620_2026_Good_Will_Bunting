package org.usfirst.frc3620.odo;

public class OdoButtonId implements IOdoButtonId {
  int id;

  public OdoButtonId(int id) {
    this.id = id;
  }

  @Override
  public int getButtonNumber() {
    return id;
  }
}