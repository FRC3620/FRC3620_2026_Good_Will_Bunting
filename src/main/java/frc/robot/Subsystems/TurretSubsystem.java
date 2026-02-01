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
import com.ctre.phoenix6.hardware.TalonFXS;

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
import yams.motorcontrollers.remote.TalonFXSWrapper;
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
        telemetryPrefix + "Subsystem") || RobotContainer.shouldMakeAllCANDevices();
    if (makeDevices) {
      motor = new TalonFX(motorId);

      SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.CLOSED_LOOP)
          .withClosedLoopController(4, 0, 0, DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90))
          // Configure Motor and Mechanism properties
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
          .withIdleMode(MotorMode.BRAKE)
          .withMotorInverted(false)
          // Setup Telemetry
          .withTelemetry(telemetryPrefix + "Motor", TelemetryVerbosity.HIGH)
          // Power Optimization
          .withStatorCurrentLimit(Amps.of(40))
          .withClosedLoopRampRate(Seconds.of(0.25))
          .withOpenLoopRampRate(Seconds.of(0.25));

      smartMotorController = new TalonFXWrapper(motor,
          DCMotor.getKrakenX60(1),
          motorConfig);

      PivotConfig pivot_config = new PivotConfig(smartMotorController)
          .withStartingPosition(Degrees.of(0)) // Starting position of the Pivot
          .withHardLimit(Degrees.of(-135), Degrees.of(135)) // Hard limit bc wiring prevents infinite spinning
          .withTelemetry("PivotExample", TelemetryVerbosity.HIGH) // Telemetry
          .withMOI(Meters.of(0.25), Pounds.of(2)); // MOI Calculation

      pivot = new Pivot(pivot_config);

    }
  }

  private Command doNothingCommand() {
    return run(() -> {
      // Don't do anything
    });
  }

  public Command setAngle(Angle angle) {
    if (pivot == null) {
      return doNothingCommand();
    } else {
      return pivot.setAngle(angle);
    }
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
