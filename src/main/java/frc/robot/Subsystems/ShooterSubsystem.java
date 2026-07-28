// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.TreeMap;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.usfirst.frc3620.CANDeviceType;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.Idle;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Helpers.AllianceFlipUtil;
import frc.robot.Helpers.ParadeShotCalculator;
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
import edu.wpi.first.wpilibj.DigitalInput;

@SuppressWarnings("unused")
public class ShooterSubsystem extends SubsystemBase {
  int motorId1 = Constants.MOTORID_SHOOTER1;
  int motorId2 = Constants.MOTORID_SHOOTER2;
  String telemetryPrefix = "Shooter";

  private DigitalInput beamBreak;

  private SysIdRoutine sysIdRoutine;

  private TalonFX motor1 = null;
  private TalonFX motor2 = null;
  private SmartMotorController smartMotorController = null;
  private FlyWheel flywheel = null;

  private AngularVelocity filteredRPM = RPM.of(0);

  private TreeMap<Integer, Double> rpmCorrectionMap = new TreeMap<>();

  private double learningRate = 0.25;
  private Distance bucketSize = Feet.of(1); // ft range for each bucket in the correction map

  private boolean atRPM = false;

  private static int shotsFiredCounter = 0;
  private boolean lastBeamBreakState = false;
  private boolean currentBeamBreakState = false;
  private boolean shotJustFired = false;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {

    rpmCorrectionMap.put(4, -52.0);
    rpmCorrectionMap.put(5, -35.0);
    rpmCorrectionMap.put(6, -30.0);
    rpmCorrectionMap.put(7, -50.0);
    rpmCorrectionMap.put(8, -35.0);
    rpmCorrectionMap.put(9, -20.0);
    rpmCorrectionMap.put(10, 0.0);
    rpmCorrectionMap.put(11, 30.0);
    rpmCorrectionMap.put(12, 30.0);
    rpmCorrectionMap.put(13, 30.0);
    rpmCorrectionMap.put(14, 30.0);
    rpmCorrectionMap.put(15, 30.0);
    rpmCorrectionMap.put(16, 60.0);
    rpmCorrectionMap.put(17, 170.0);
    rpmCorrectionMap.put(18, 280.0);
    rpmCorrectionMap.put(19, 320.0);
    rpmCorrectionMap.put(20, 320.0);
    rpmCorrectionMap.put(21, 320.0);
    rpmCorrectionMap.put(22, 320.0);
    rpmCorrectionMap.put(23, 320.0);
    rpmCorrectionMap.put(24, 320.0);
    rpmCorrectionMap.put(25, 320.0);
    rpmCorrectionMap.put(26, 320.0);
    rpmCorrectionMap.put(27, 500.0);
    rpmCorrectionMap.put(28, 500.0);
    rpmCorrectionMap.put(29, 500.0);
    rpmCorrectionMap.put(30, 500.0);
    rpmCorrectionMap.put(31, 500.0);
    rpmCorrectionMap.put(32, 500.0);
    rpmCorrectionMap.put(33, 500.0);
    rpmCorrectionMap.put(34, 500.0);
    rpmCorrectionMap.put(35, 500.0);
    rpmCorrectionMap.put(36, 500.0);
    rpmCorrectionMap.put(37, 500.0);
    rpmCorrectionMap.put(38, 500.0);
    rpmCorrectionMap.put(39, 500.0);
    rpmCorrectionMap.put(40, 500.0);
    rpmCorrectionMap.put(41, 500.0);

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
          // .withClosedLoopController(0.25, 0, 0.45, RPM.of(3500),
          // RotationsPerSecondPerSecond.of(58.3))
          // .withSimClosedLoopController(2, 0, 0.0, RPM.of(3500),
          // RotationsPerSecondPerSecond.of(58.3))
          // Feedforward Constants
          .withFeedforward(new SimpleMotorFeedforward(0.30179, 0.24115, 0.016414))
          .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
          // Telemetry name and verbosity level
          .withTelemetry("motor1", TelemetryVerbosity.LOW)
          // Gearing from the motor rotor to final shaft.
          // In this example gearbox(3,4) is the same as gearbox("3:1","4:1") which
          // corresponds to the gearbox attached to your motor.
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(2)))
          // Motor properties to prevent over currenting.
          .withMotorInverted(false)
          .withIdleMode(MotorMode.COAST)
          .withMechanismCircumference(Inches.of(Math.PI * 4))
          .withStatorCurrentLimit(Amps.of(40))
          .withClosedLoopRampRate(Seconds.of(0.5))
          .withOpenLoopRampRate(Seconds.of(0.5));

      smartMotorController = new TalonFXWrapper(motor1, DCMotor.getKrakenX60(1), smcConfig1);

      /*
       * FlyWheelConfig Config = new FlyWheelConfig(smartMotorController)
       * // Diameter of the flywheel.
       * .withDiameter(Inches.of(4))
       * // Mass of the flywheel.
       * .withMass(Pounds.of(1.5))
       * // Maximum speed of the flywheel.
       * .withUpperSoftLimit(RPM.of(5000))
       * // Telemetry name and verbosity for the arm.
       * .withTelemetry(telemetryPrefix, TelemetryVerbosity.LOW);
       */
      // flywheel = new FlyWheel(Config);

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
                SignalLogger.writeDouble("Shooter_Position_Rotations",
                    smartMotorController.getMechanismPosition().in(Rotations));
              },
              null,
              this));

      setDefaultCommand(idle());
    }

    beamBreak = new DigitalInput(8);

    SmartDashboard.putNumber("frc3620/Shooter/Flywheel RPM Dashboard Control", 0);
    SmartDashboard.putNumber("frc3620/Shooter/Filtering Alpha", 1.0);

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

    return idle();// flywheel.setSpeed(speed).withName(telemetryPrefix + " SetVelocity");
  }

  public Command setVelocityDashboardCommand() {
    if (flywheel == null)
      return idle();

    return createSetVelocityCommand(
        () -> RPM.of(SmartDashboard.getNumber("frc3620/Shooter/Flywheel RPM Dashboard Control", 0)));

  }

  public Command createSetSpeedToTargetCommand(Supplier<Translation3d> targetPosition, Supplier<Pose2d> robotPosition,
      Supplier<VelocityVector> robotVelocity) {
    if (flywheel == null)
      return idle();

    return idle();/*
                   * flywheel.setSpeed(
                   * () -> {
                   * AngularVelocity raw = ShotCalculator.calculateShooterSpeed(targetPosition,
                   * robotPosition, robotVelocity);
                   * 
                   * Distance distanceFeet = getDistanceToTarget(() ->
                   * targetPosition.get().toTranslation2d(), robotPosition);
                   * 
                   * double correctionRPM = getRPMCorrection(distanceFeet);
                   * 
                   * AngularVelocity corrected = raw.plus(RPM.of(correctionRPM));
                   * 
                   * SmartDashboard.putNumber("frc3620/Shooter/DistanceFeet",
                   * distanceFeet.in(Feet));
                   * SmartDashboard.putNumber("frc3620/Shooter/RPMCorrection", correctionRPM);
                   * return corrected;
                   * });
                   */
  }

  public Command paradeAutoAim() {
    if (flywheel == null)
      return idle();

    /*
     * return flywheel.setSpeed(() -> {
     * var dist = RobotContainer.limelightSubsystem.getDistanceToTag22();
     * if (dist.isEmpty()) {
     * SmartDashboard.putString("frc3620/Shooter/ParadeStatus", "No Tag");
     * return filteredRPM; // hold last known speed
     * }
     * AngularVelocity speed =
     * ParadeShotCalculator.calculateShooterSpeed(Meters.of(dist.get()));
     * filteredRPM = speed; // keep filteredRPM updated so atRPM() stays meaningful
     * SmartDashboard.putString("frc3620/Shooter/ParadeStatus", "Tracking");
     * return speed;
     * }).withName(telemetryPrefix + " paradeAutoAim");
     */
    return idle();
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
      SmartDashboard.putNumber(
          "frc3620/Shooter/CorrectionMap/LearnedPoints",
          rpmCorrectionMap.size());
      for (var entry : rpmCorrectionMap.entrySet()) {
        SmartDashboard.putNumber("frc3620/Shooter/CorrectionMap/" + entry.getKey(),
            entry.getValue());
      }
      SmartDashboard.putBoolean("frc3620/Shooter/atRPM", atRPM());
      SmartDashboard.putNumber("frc3620/Shooter/CorrectionAtCurrentDistance", getRPMCorrection(getDistanceToTarget(
          () -> ShotCalculator.FieldTargets.BLUE_HUB.getTargetPositionSupplier().get().toTranslation2d(),
          () -> AllianceFlipUtil.apply(RobotContainer.swerveSubsystem.getState().Pose))));
    }

    if (beamBreak != null) {

      currentBeamBreakState = !beamBreak.get();

      shotJustFired = lastBeamBreakState && !currentBeamBreakState;

      if (shotJustFired) {
        shotsFiredCounter++;
      }
      lastBeamBreakState = currentBeamBreakState;

      SmartDashboard.putBoolean("frc3620/Shooter/BeamBreakBroken", currentBeamBreakState);
      SmartDashboard.putNumber("frc3620/Shooter/ShotsFiredCounter", shotsFiredCounter);
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

  private double getRPMCorrection(Distance distance) {

    if (rpmCorrectionMap.isEmpty()) {
      return 0;
    }

    double distBuckets = distance.in(Feet) / bucketSize.in(Feet);

    Integer lowKey = rpmCorrectionMap.floorKey((int) Math.floor(distBuckets));
    Integer highKey = rpmCorrectionMap.ceilingKey((int) Math.ceil(distBuckets));

    if (highKey == null && lowKey == null)
      return 0;
    if (lowKey == null)
      return rpmCorrectionMap.get(highKey);
    if (highKey == null)
      return rpmCorrectionMap.get(lowKey);

    if (lowKey.equals(highKey))
      return rpmCorrectionMap.get(lowKey);

    double lowerDist = lowKey * bucketSize.in(Feet);
    double highDist = highKey * bucketSize.in(Feet);

    double lowerVal = rpmCorrectionMap.get(lowKey);
    double upperVal = rpmCorrectionMap.get(highKey);

    double t = (distance.in(Feet) - lowerDist) / (highDist - lowerDist);

    return lowerVal * (1 - t) + upperVal * t;
  }

  public void learnShot(Distance distance, double rpmAdjustment) {

    Integer bucket = getBucket(distance);

    double current = rpmCorrectionMap.getOrDefault(bucket, 0.0);

    double updated = current + learningRate * rpmAdjustment;

    SmartDashboard.putNumber("DELETEMELATER/bucketToGoInFt", bucket);
    SmartDashboard.putNumber("DELETEMELATER/CurrentRPMOffset", current);
    SmartDashboard.putNumber("DELETEMELATER/UpdatedRPMOffset", updated);
    SmartDashboard.putNumber("DELETEMELATER/DistanceInput", distance.in(Feet));
    SmartDashboard.putNumber("DELETEMELATER/BucketSize", bucketSize.in(Feet));

    rpmCorrectionMap.put(bucket, updated);
  }

  private Distance getDistanceToTarget(Supplier<Translation2d> targetPosition, Supplier<Pose2d> robotPosition) {
    return ShotCalculator.calculateBaseHDistanceToTarget(targetPosition, robotPosition);
  }

  private Integer getBucket(Distance distance) {
    return (int) Math.floor(distance.in(Feet) / bucketSize.in(Feet));
  }

  public boolean atRPM() {

    AngularVelocity current = getVelocity();
    if (current.isNear(filteredRPM, RPM.of(500))) {
      atRPM = true;
    } else {
      atRPM = false;
    }

    return atRPM;
  }

  public TalonFX getMotor1() {
    if (motor1 != null) {
      return motor1;
    }
    return null;
  }

  public TalonFX getMotor2() {
    if (motor2 != null) {
      return motor2;
    }
    return null;
  }

  public BooleanSupplier shotFired() {

    return () -> shotJustFired;
  }
}