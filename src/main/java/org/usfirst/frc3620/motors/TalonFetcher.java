package org.usfirst.frc3620.motors;

import com.ctre.phoenix6.hardware.TalonFX;

class TalonFetcher extends MotorWatcherFetcher {
  TalonFX talonFX;

  TalonFetcher(TalonFX _talonFX) {
    super();
    talonFX = _talonFX;
  }

  @Override
  Double measureTemperature() {
    return temperature = talonFX.getDeviceTemp().getValueAsDouble();
  }

  @Override
  Double measurePosition() {
    return position = talonFX.getPosition().getValueAsDouble();
  }

  @Override
  Double measureOutputCurrent() {
    return outputCurrent = talonFX.getStatorCurrent().getValueAsDouble();
  }
}
