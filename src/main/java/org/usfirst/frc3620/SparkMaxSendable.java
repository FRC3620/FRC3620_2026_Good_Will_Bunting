package org.usfirst.frc3620;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;

public class SparkMaxSendable extends SparkMax implements Sendable  {
    public SparkMaxSendable(int deviceId, MotorType motorType) {
        super(deviceId, motorType);

        SendableRegistry.addLW(this, "SparkMax", deviceId);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Motor Controller");
        builder.setActuator(true);
        builder.setSafeState(this::stopMotor);
        builder.addDoubleProperty("Value", this::get, this::set);
    }
}