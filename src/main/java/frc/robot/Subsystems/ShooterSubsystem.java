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
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import org.usfirst.frc3620.CANDeviceType;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Helpers.ShotCalculator;
import frc.robot.Helpers.VelocityVector;
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

  private SysIdRoutine sysIdRoutine;

  private TalonFX motor1 = null;
  private TalonFX motor2 = null;
  private SmartMotorController smartMotorController = null;
  private FlyWheel flywheel = null;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {

    boolean makeDevices = RobotContainer.canDeviceFinder.isDevicePresent(CANDeviceType.TALON_PHOENIX6, motorId1,
        telemetryPrefix + " #1") || RobotContainer.shouldMakeAllCANDevices();
    if (makeDevices) {
      motor1 = new TalonFX(motorId1);
      RobotContainer.canDeviceFinder.isDevicePresent(CANDeviceType.TALON_PHOENIX6, motorId2,
          telemetryPrefix + " #2");
      motor2 = new TalonFX(motorId2);
      RobotContainer.healthSubsystem.addMotorToWatch(motor1, telemetryPrefix + "#1",
          HealthSubsystem.healthOptionsForYAMS);
      RobotContainer.healthSubsystem.addMotorToWatch(motor2, telemetryPrefix + "#2",
          HealthSubsystem.healthOptionsForYAMS);

      SmartMotorControllerConfig smcConfig1 = new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.CLOSED_LOOP)
          .withFollowers(Pair.of(motor2, true)) // motor2 follows motor1, inverted
          // Feedback Constants (PID Constants)
          .withClosedLoopController(0.2, 0, 0.0, DegreesPerSecond.of(14400), DegreesPerSecondPerSecond.of(14400))
          .withSimClosedLoopController(10, 0, 0, DegreesPerSecond.of(360), DegreesPerSecondPerSecond.of(180))
          // Feedforward Constants
          .withFeedforward(new SimpleMotorFeedforward(0.30179, 0.24115, 0.016414))
          .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
          // Telemetry name and verbosity level
          .withTelemetry("motor1", TelemetryVerbosity.HIGH)
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

      sysIdRoutine = new SysIdRoutine(
          new SysIdRoutine.Config(
              null,
              null,
              null,
              (state) -> SignalLogger.writeString("Shooter_State", state.toString())),
          new SysIdRoutine.Mechanism(
              (voltage) -> {
                smartMotorController.stopClosedLoopController();
                smartMotorController.setVoltage(voltage);

                SignalLogger.writeDouble("Shooter Voltage", voltage.in(Volts));
                SignalLogger.writeDouble("Shooter_Velocity_RPS", getVelocity().in(RotationsPerSecond));
                SignalLogger.writeDouble("Shooter_Position_Rotations", smartMotorController.getMechanismPosition().in(Rotations));
              },
              null,
              this));

      setDefaultCommand(idle());
    }
    SmartDashboard.putNumber("frc3620/Shooter/Flywheel RPM Dashboard Control", 0);

  }

  /**
   * Gets the current velocity of the flywheel.
   *
   * @return Flywheel velocity.
   */
  public AngularVelocity getVelocity() {
    if (flywheel == null)
      return RPM.of(999999);
    else
      return flywheel.getSpeed();
  }

  /**
   * Set the flywheel velocity.
   *
   * @param speed Speed to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command createSetVelocityCommand(Supplier<AngularVelocity> speed) {
    if (flywheel == null)
      return idle();

    return flywheel.setSpeed(speed.get()).withName(telemetryPrefix + " SetVelocity");
  }

  public Command setVelocityDashboardCommand() {
    if (flywheel == null)
      return idle();

      return createSetVelocityCommand(() -> RPM.of(SmartDashboard.getNumber("frc3620/Shooter/Flywheel RPM Dashboard Control", 0)));

  }

  public Command createSetSpeedToTargetCommand(Translation3d targetPosition, Supplier<Pose2d> robotPosition, Supplier<VelocityVector> robotVelocity) {
    if (flywheel == null)
      return idle();

    return flywheel.setSpeed(() -> ShotCalculator.calculateShooterSpeed(targetPosition, robotPosition, robotVelocity)).withName(telemetryPrefix + " SetSpeedToTarget");
  }

  /**
   * Set the dutycycle of the flywheel.
   *
   * @param dutyCycle DutyCycle to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command set(double dutyCycle) {
    Command rv;
    if (flywheel != null) {
      rv = flywheel.set(dutyCycle);
    } else {
      rv = idle();
    }
    return rv.withName(telemetryPrefix + " Set");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (flywheel != null) {
      flywheel.updateTelemetry();
      SmartDashboard.putNumber("frc3620/" + telemetryPrefix + "/RPM Actual", getVelocity().in(RPM));
    }
  }

  @Override
  public void simulationPeriodic() {
    if (flywheel != null) {
      flywheel.simIterate();
    }
  }

  public Command sysIdQuasistaticForward() {
    return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward);
  }

  public Command sysIdQuasistaticReverse() {
    return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse);
  }

  public Command sysIdDynamicForward() {
    return sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward);
  }

  public Command sysIdDynamicReverse() {
    return sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse);
  }

}
