package org.usfirst.frc3620.odo;

public class OdoAxisId implements IOdoAxisId {
  int id;

  public OdoAxisId(int id) {
    this.id = id;
  }

  @Override
  public int getAxisNumber() {
    return id;
  }
}