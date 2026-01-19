package org.usfirst.frc3620.motors;

import com.revrobotics.spark.SparkMax;

class SparkMAXFetcher extends MotorWatcherFetcher {
  SparkMax sparkMax;

  SparkMAXFetcher(SparkMax _sparkMAX) {
    super();
    sparkMax = _sparkMAX;
  }

  @Override
  Double measureTemperature() {
    return temperature = sparkMax.getMotorTemperature();
  }

  @Override
  Double measurePosition() {
    return position = sparkMax.getAbsoluteEncoder().getPosition();
  }

  @Override
  Double measureOutputCurrent() {
    return outputCurrent = sparkMax.getOutputCurrent();
  }
}
