// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Seconds;

import org.usfirst.frc3620.CANDeviceType;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.positional.Elevator;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class ClimberSubsystem extends SubsystemBase {
  int motorId = Constants.MOTORID_CLIMBER;
  String telemetryPrefix = "Climber";

  private TalonFX motor = null;
  SmartMotorController motorController = null;
  private Elevator elevator = null;

  public ClimberSubsystem() {

    boolean makeDevice = RobotContainer.canDeviceFinder.isDevicePresent(CANDeviceType.TALON_PHOENIX6,
        motorId,telemetryPrefix) || RobotContainer.shouldMakeAllCANDevices();
    if (makeDevice) {
      motor = new TalonFX(motorId);
      RobotContainer.healthSubsystem.addMotorToWatch(motor, telemetryPrefix, HealthSubsystem.healthOptionsForYAMS);

      SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.CLOSED_LOOP)
          // Mechanism Circumference is the distance traveled by each mechanism rotation
          // converting rotations to meters.
          // .withMechanismCircumference(Meters.of(Inches.of(0.25).in(Meters) * 22))
          .withMechanismCircumference(
              Inches.of(Math.PI * 0.25))

          // Feedback Constants (PID Constants)
          .withClosedLoopController(4, 0, 0, MetersPerSecond.of(0.5), MetersPerSecondPerSecond.of(0.5))
          .withSimClosedLoopController(4, 0, 0, MetersPerSecond.of(0.5), MetersPerSecondPerSecond.of(0.5))
          // Feedforward Constants
          .withFeedforward(new ElevatorFeedforward(0, 0, 0))
          .withSimFeedforward(new ElevatorFeedforward(0, 0, 0))
          // Telemetry name (relative to the mechanism telemetry name) and verbosity level
          .withTelemetry("motor", TelemetryVerbosity.HIGH)
          // Gearing from the motor rotor to final shaft.
          // In this example gearbox(3,4) is the same as gearbox("3:1","4:1") which
          // corresponds to the gearbox attached to your motor.
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
          // Motor properties to prevent over currenting.
          .withMotorInverted(false)
          .withIdleMode(MotorMode.BRAKE)
          .withStatorCurrentLimit(Amps.of(40))
          .withClosedLoopRampRate(Seconds.of(0.25))
          .withOpenLoopRampRate(Seconds.of(0.25));

      motorController = new TalonFXWrapper(motor,
          DCMotor.getKrakenX60(1),
          motorConfig);

      elevator = new Elevator(new ElevatorConfig(motorController)
          .withStartingHeight(Meters.of(0.5))
          .withHardLimits(Meters.of(0), Meters.of(3))
          .withTelemetry(telemetryPrefix, TelemetryVerbosity.HIGH)
          .withMechanismPositionConfig(
              new MechanismPositionConfig()
                  .withMaxRobotHeight(Inches.of(30))
                  .withMaxRobotLength(Inches.of(34))
                  .withRelativePosition(new Translation3d(Inches.of(0), Inches.of(0), Inches.of(0))))
          .withMass(Pounds.of(16)));
    }
  }

  /**
   * Set the height of the elevator.
   * 
   * @param angle Distance to go to.
   */
  public Command setHeight(Distance height) {
    Command rv;
    if (elevator == null) {
      rv = idle();
    } else {
      rv = elevator.setHeight(height);
    }
    return rv.withName(telemetryPrefix + " SetHeight");
  }

  /**
   * Move the elevator up and down.
   * 
   * @param dutycycle [-1, 1] speed to set the elevator too.
   */
  public Command set(double dutycycle) {
    Command rv;
    if (elevator == null) {
      rv = idle();
    } else {
      rv = elevator.set(dutycycle);
    }
    return rv.withName(telemetryPrefix + " Set");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (elevator != null) {
      elevator.updateTelemetry();
      elevator.getMechanismSetpoint().ifPresent((setpoint) -> SmartDashboard.putNumber(
          "frc3620/" + telemetryPrefix + "/setPos",
          setpoint.in(edu.wpi.first.units.Units.Radians)));

      SmartDashboard.putNumber("frc3620/" + telemetryPrefix + "/actualPos", getPosition());

    }
  }

  public double getPosition() {
    if (elevator != null) {
      return elevator.getHeight().in(Meters);
    } else {
      return 0;
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    if (elevator != null) {
      elevator.simIterate();
    }
  }

}
