package org.usfirst.frc3620.motors;

import java.util.function.Consumer;
import java.util.function.Function;

public enum MotorWatcherMetric {

    TEMPERATURE("temperature", (m) -> m.measureTemperature(), (m) -> m.getTemperature()),
    POSITION("position", (m) ->m.measurePosition(), (m) -> m.getPosition()),
    OUTPUT_CURRENT("outputCurrent", (m) -> m.measureOutputCurrent(), (m) -> m.getOutputCurrent()),
    ;
     
    final String name;
    final Consumer<MotorWatcherFetcher> measurer;
    final Function<MotorWatcherFetcher, Double> getter;

    MotorWatcherMetric(String _name, Consumer<MotorWatcherFetcher> _measurer, Function<MotorWatcherFetcher,Double> _getter) {
        name = _name;
        measurer = _measurer;
        getter = _getter;
    }

    public String getName() {
        return name;
    }

    public void measure(MotorWatcherFetcher c) {
        measurer.accept(c);
    }
    
    public Double getValue(MotorWatcherFetcher values) {
        return getter.apply(values);
    };

}
