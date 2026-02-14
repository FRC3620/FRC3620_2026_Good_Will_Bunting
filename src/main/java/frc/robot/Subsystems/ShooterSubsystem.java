// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import org.usfirst.frc3620.CANDeviceType;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

@SuppressWarnings("unused")
public class ShooterSubsystem extends SubsystemBase {
  int motorId1 = Constants.MOTORID_SHOOTER1;
  int motorId2 = Constants.MOTORID_SHOOTER2;
  String telemetryPrefix = "Shooter";

  private TalonFX motor1 = null;
  private TalonFX motor2 = null;
  private SmartMotorController smartMotorController = null;
  private FlyWheel flywheel = null;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {

    boolean makeDevices = RobotContainer.canDeviceFinder.isDevicePresent(CANDeviceType.TALON_PHOENIX6, motorId1,
        telemetryPrefix + "Subsystem") || RobotContainer.shouldMakeAllCANDevices();
    if (makeDevices) {
      motor1 = new TalonFX(motorId1);
      motor2 = new TalonFX(motorId2);

      SmartMotorControllerConfig smcConfig1 = new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.CLOSED_LOOP)
          .withFollowers(Pair.of(motor2, true)) // motor2 follows motor1, inverted
          // Feedback Constants (PID Constants)
          .withClosedLoopController(10, 0, 0.1, DegreesPerSecond.of(14400), DegreesPerSecondPerSecond.of(14400))
          .withSimClosedLoopController(10, 0, 0, DegreesPerSecond.of(360), DegreesPerSecondPerSecond.of(180))
          // Feedforward Constants
          .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
          .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
          // Telemetry name and verbosity level
          .withTelemetry(telemetryPrefix + "Motor1", TelemetryVerbosity.HIGH)
          // Gearing from the motor rotor to final shaft.
          // In this example gearbox(3,4) is the same as gearbox("3:1","4:1") which
          // corresponds to the gearbox attached to your motor.
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(2)))
          // Motor properties to prevent over currenting.
          .withMotorInverted(false)
          .withIdleMode(MotorMode.COAST)
          .withStatorCurrentLimit(Amps.of(40))
          .withClosedLoopRampRate(Seconds.of(0.5))
          .withOpenLoopRampRate(Seconds.of(0.5));


      smartMotorController = new TalonFXWrapper(motor1, DCMotor.getKrakenX60(1), smcConfig1);
    }

    FlyWheelConfig Config = new FlyWheelConfig(smartMotorController)
        // Diameter of the flywheel.
        .withDiameter(Inches.of(4))
        // Mass of the flywheel.
        .withMass(Pounds.of(0.6))
        // Maximum speed of the flywheel.
        .withUpperSoftLimit(RPM.of(5000))
        // Telemetry name and verbosity for the arm.
        .withTelemetry(telemetryPrefix, TelemetryVerbosity.HIGH);
    flywheel = new FlyWheel(Config);
  }

  /**
   * Gets the current velocity of the flywheel.
   *
   * @return Flywheel velocity.
   */
  public AngularVelocity getVelocity() {
    if (flywheel == null)
      return RPM.of(50);
    else
      return flywheel.getSpeed();
  }

  /**
   * Set the flywheel velocity.
   *
   * @param speed Speed to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command setVelocity(AngularVelocity speed) {
    if (flywheel != null)
      return flywheel.setSpeed(speed);
    else
      return this.runOnce(() -> {
        // RobotContainer.logger.error("flywheel not initialized");
      });
  }

  public void setVelocityDirect(AngularVelocity speed) {
    if(flywheel != null) {
      flywheel.setSpeed(speed);
    }
  }

  /**
   * Set the dutycycle of the flywheel.
   *
   * @param dutyCycle DutyCycle to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command set(double dutyCycle) {
    if (flywheel != null)
      return flywheel.set(dutyCycle);
    else
      return this.runOnce(() -> {
        // RobotContainer.logger.error("Flywheel not initialized");
      });
  }




  @Override
  public void periodic() {
    // This method will be called once per scheduler run
   if (flywheel != null) {
      flywheel.updateTelemetry();}
   }

  @Override
  public void simulationPeriodic() {
     if (flywheel != null) {
      flywheel.simIterate();
    }
  }
}

