// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Seconds;

import org.usfirst.frc3620.CANDeviceType;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class TurretSubsystem extends SubsystemBase {
  int motorId = Constants.MOTORID_TURRET;
  String telemetryPrefix = "Turret";

  private TalonFX motor = null;
  private SmartMotorController smartMotorController = null;
  private Pivot pivot = null;

  /** Creates a new TurretSubsystem. */
  public TurretSubsystem() {
    boolean makeDevices = RobotContainer.canDeviceFinder.isDevicePresent(CANDeviceType.TALON_PHOENIX6, motorId,
        telemetryPrefix) || RobotContainer.shouldMakeAllCANDevices();
    if (makeDevices) {
      motor = new TalonFX(motorId);
      RobotContainer.healthSubsystem.addMotorToWatch(motor, telemetryPrefix);

      SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.CLOSED_LOOP)
          .withClosedLoopController(4, 0, 0, DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90))
          // Configure Motor and Mechanism properties
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
          .withIdleMode(MotorMode.BRAKE)
          .withMotorInverted(false)
          // Setup Telemetry
          .withTelemetry("motor", TelemetryVerbosity.HIGH)
          // Power Optimization
          .withStatorCurrentLimit(Amps.of(40))
          .withClosedLoopRampRate(Seconds.of(0.25))
          .withOpenLoopRampRate(Seconds.of(0.25));

      smartMotorController = new TalonFXWrapper(motor,
          DCMotor.getKrakenX60(1),
          motorConfig);

      pivot = new Pivot(new PivotConfig(smartMotorController)
          // Starting position of the Pivot
          .withStartingPosition(Degrees.of(0))
          // Hard limit bc wiring prevents infinite spinning
          .withHardLimit(Degrees.of(-135), Degrees.of(135))
          // Telemetry
          .withTelemetry(telemetryPrefix, TelemetryVerbosity.HIGH)
          // MOI Calculation
          .withMOI(Meters.of(0.25), Pounds.of(2)));
    }
  }

  public Command setAngle(Angle angle) {
    Command rv;
    if (pivot == null) {
      rv = idle();
    } else {
      rv = pivot.setAngle(angle);
    }
    return rv.withName(telemetryPrefix + " setAngle");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (pivot != null) {
      pivot.updateTelemetry();
    }
  }

  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    if (pivot != null) {
      pivot.simIterate();
    }
  }
}
