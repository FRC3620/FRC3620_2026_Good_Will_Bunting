package frc.robot;

import org.usfirst.frc3620.RobotParametersBase;

/**
 * add members here as needed
 */
public class RobotParameters extends RobotParametersBase {
    String variant = "Other";   // default value if not specified in JSON file

    public String getVariant() {
        return variant;
    }
}