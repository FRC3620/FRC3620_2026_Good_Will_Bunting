package org.usfirst.frc3620.motors;

import java.util.*;

/**
 * if you add fields to this, make sure you add to MotorWatcherMetric!
 */
abstract class MotorWatcherFetcher {
  Double temperature;
  Double position;
  Double outputCurrent;

  /**
   * this should measure, save, and return the motor temperature.
   * @return motor temp (Celsius)
   */
  abstract Double measureTemperature();

  /**
   * this returns the last measured temperature
   * @return last measured temperature
   */
  final Double getTemperature() {
    return temperature;
  };

  /**
   * measure, save, and return the motor position.
   * @return measured position
   */
  abstract Double measurePosition();

  /**
   * this returns the last measured position.
   * @return last measured position
   */
  final Double getPosition() {
    return position;
  }

  /**
   * measure, save, and return the output current.
   * @return measured output current
   */
  abstract Double measureOutputCurrent();

  /**
   * return the last measured output current.
   * @return last measured output current
   */
  final Double getOutputCurrent() {
    return outputCurrent;
  };

  /**
   * override this if needed. this would be used in the case where some work
   * needs to be done once per cycle to do measurements. 
   */
  void startMeasurements() {
  }

  /**
   * override this if needed
   */
  void finalizeMeasurements() {
  }

  void collect(EnumSet<MotorWatcherMetric> metrics) {
    startMeasurements();
    for (var metric : metrics) {
      metric.measure(this);
    }
    finalizeMeasurements();
  }
}
