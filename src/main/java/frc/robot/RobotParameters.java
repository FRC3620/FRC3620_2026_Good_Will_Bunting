package frc.robot;

import java.util.ArrayList;
import java.util.List;

import org.usfirst.frc3620.RobotParametersBase;

import com.fasterxml.jackson.annotation.JsonFormat;

/**
 * add members here as needed
 */
public class RobotParameters extends RobotParametersBase {
    String variant = "Other";   // default value if not specified in JSON file
    List<String> nonCriticalCANDevices = new ArrayList<>();
    List<String> ignoreHealth = new ArrayList<>();
    List<Integer> breakersToIgnore = new ArrayList<>();

    public RobotParameters() {
        super();
        for (int channel = 0; channel < 24; channel++) {
            breakersToIgnore.add(channel);
        }
    }

    public String getVariant() {
        return variant;
    }

    @JsonFormat(with = JsonFormat.Feature.ACCEPT_SINGLE_VALUE_AS_ARRAY)
    public List<String> getNonCriticalCANDevices() {
        return nonCriticalCANDevices;
    }

    @JsonFormat(with = JsonFormat.Feature.ACCEPT_SINGLE_VALUE_AS_ARRAY)
    public List<String> getIgnoreHealth() {
        return ignoreHealth;
    }

    @Override
    public String toString() {
        return "RobotParameters [variant=" + variant + ", nonCriticalCANDevices=" + nonCriticalCANDevices
                + ", ignoreHealth=" + ignoreHealth + ", breakersToIgnore=" + breakersToIgnore + ", name=" + name
                + ", makeAllCANDevices=" + makeAllCANDevices + "]";
    }

    public List<Integer> getBreakersToIgnore() {
        return breakersToIgnore;
    }
}