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
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.Supplier;

import org.usfirst.frc3620.CANDeviceType;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Helpers.ShotCalculator;
import frc.robot.Helpers.VelocityVector;
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
import yams.units.EasyCRT;
import yams.units.EasyCRTConfig;

public class TurretSubsystem extends SubsystemBase {
  int motorId = Constants.MOTORID_TURRET;

  String telemetryPrefix = "Turret";

  private TalonFX motor = null;
  private CANcoder turretEncoderA = null;
  private CANcoder turretEncoderB = null;

  private final EasyCRTConfig easyCrtConfig;
  private boolean rotorSeededFromAbs = false;
  private double lastSeededTurretDeg = Double.NaN;
  private double lastSeedError = Double.NaN;
  private double lastAbsA = Double.NaN;
  private double lastAbsB = Double.NaN;
  private String lastSeedStatus = "NOT_ATTEMPTED";

  private Timer startUpTimer = new Timer();
  private boolean startTimer = false;
  private boolean delayForCRTDone = false;

  /** Manually rerun CRT seeding. */
  private static final String RERUN_SEED = "Turret/CRT/RerunSeed";

  private SmartMotorController smartMotorController = null;
  private Pivot pivot = null;

  private static final Angle absAEncoderOffset = Rotations.of(-0.600341796875);
  private static final Angle absBEncoderOffset = Rotations.of(-0.574462890625);

  /** Creates a new TurretSubsystem. */
  public TurretSubsystem() {
    boolean makeDevices = RobotContainer.canDeviceFinder.isDevicePresent(CANDeviceType.TALON_PHOENIX6, motorId,
        telemetryPrefix) || RobotContainer.shouldMakeAllCANDevices();
    if (makeDevices) {
      motor = new TalonFX(motorId);

      turretEncoderA = new CANcoder(Constants.ENCODERID_TURRET_A);
      turretEncoderB = new CANcoder(Constants.ENCODERID_TURRET_B);

      RobotContainer.healthSubsystem.addMotorToWatch(motor, telemetryPrefix, HealthSubsystem.healthOptionsForYAMS);

      SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.CLOSED_LOOP)
          .withClosedLoopController(75, 0, 0, DegreesPerSecond.of(2500), DegreesPerSecondPerSecond.of(2500))
          // Configure Motor and Mechanism properties
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(50.0/14.0,140.0/18.0)))
          //.withContinuousWrapping(Degrees.of(0), Degrees.of(360))
          .withIdleMode(MotorMode.BRAKE)
          .withMotorInverted(true)
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
          //.withWrapping(Degrees.of(0), Degrees.of(360))
          // Hard limit bc wiring prevents infinite spinning
          .withHardLimit(Degrees.of(-232), Degrees.of(128))
          .withSoftLimits(Degrees.of(-232), Degrees.of(128))
          // Telemetry
          .withTelemetry(telemetryPrefix, TelemetryVerbosity.HIGH)
          // MOI Calculation
          .withMOI(Meters.of(0.25), Pounds.of(2)));

      setDefaultCommand(idle());
    }

    easyCrtConfig = buildEasyCRTConfig();
    logCrtConfigTelemetry();
    SmartDashboard.putBoolean(RERUN_SEED, false);

    SmartDashboard.putNumber("frc3620/" + telemetryPrefix + "/Angle Dashboard Control", 180);
  }

  public Command createSetAngleCommand(Supplier<Angle> angle) {
    Command rv;
    Supplier<Angle> setpt = angle;
    if (pivot == null) {
      rv = idle();
    } else {
      setpt = () -> Degrees.of(MathUtil.inputModulus(angle.get().in(Degrees), -232, 128));

      rv = pivot.setAngle(setpt);
    }
    return rv.withName(telemetryPrefix + " setAngle");
  }

  public Angle getAngle() {
    if (pivot != null) {
      return pivot.getAngle();
    } else {
      return Degrees.of(9999999);
    }
  }

  public Command setAngleDashboardCommand() {
    Command rv;
    if (pivot == null) {
      rv = idle();
    } else {
      rv = createSetAngleCommand(() -> Degrees.of(SmartDashboard.getNumber("frc3620/" + telemetryPrefix + "/Angle Dashboard Control", 180)));
    }
    return rv.withName(telemetryPrefix + " setAngleDashboard");
  }

  public Command createSetAngleToTargetCommand(Translation2d targetPosition, Supplier<Pose2d> robotPose, Supplier<VelocityVector> robotVelocity) {
    Command rv;
    if (pivot == null) {
      rv = idle();
    } else {
      rv = createSetAngleCommand(() -> ShotCalculator.calculateNetTurretAngleToTarget(targetPosition, robotPose, robotVelocity));
    }
    return rv.withName(telemetryPrefix + " setAngleToTarget");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (!startTimer) {
      startUpTimer.reset();
      startUpTimer.start();
      startTimer = true;
    }

    if (pivot != null) {

      if (startUpTimer.hasElapsed(5)) {
        delayForCRTDone = true;
      }

      if (SmartDashboard.getBoolean(RERUN_SEED, false) && delayForCRTDone) {
        SmartDashboard.putBoolean(RERUN_SEED, false);
        rerunCrtSeed();
      }
      if (!rotorSeededFromAbs && delayForCRTDone) {
        attemptRotorSeedFromCANCoders();
      }
      pivot.updateTelemetry();
      SmartDashboard.putNumber("frc3620/" + telemetryPrefix + "/Angle Degrees", getAngle().in(Degrees));
      SmartDashboard.putNumber("Turret/CRT/CurrentPositionDeg",
          smartMotorController.getMechanismPosition().in(Degrees));
    }
  }

  public void rerunCrtSeed() {
    rotorSeededFromAbs = false;
    SmartDashboard.putNumber("Turret/CRT/ManualRerunTimestampSec", Timer.getFPGATimestamp());
    attemptRotorSeedFromCANCoders();
  }

  private AbsSensorRead readAbsSensors() {
    boolean haveDevices = turretEncoderA.isConnected() && turretEncoderB.isConnected();

    if (haveDevices) {
      return new AbsSensorRead(
          true,
          turretEncoderA.getAbsolutePosition().getValueAsDouble(),
          turretEncoderB.getAbsolutePosition().getValueAsDouble(),
          "OK");
    }
    return new AbsSensorRead(false, Double.NaN, Double.NaN, "NO_DEVICES");
  }

  private EasyCRTConfig buildEasyCRTConfig() {
    return new EasyCRTConfig(
        () -> turretEncoderA.getAbsolutePosition().getValue(),
        () -> turretEncoderB.getAbsolutePosition().getValue())
        // .withCommonDriveGear(1, 95, 19, 21)
        .withAbsoluteEncoder1GearingStages(140, 18)
        .withAbsoluteEncoder2GearingStages(140, 18, 24, 23)
        .withAbsoluteEncoderOffsets(absAEncoderOffset, absBEncoderOffset)
        .withAbsoluteEncoderInversions(true,true)
        .withMechanismRange(
            Rotations.of(-1), Rotations.of(1)) // 150 degrees total range
        .withMatchTolerance(Rotations.of(0.03))
        .withCrtGearRecommendationInputs(24, 140 / 18.0)
        .withCrtGearRecommendationConstraints(1.1, 11, 60, 40);
  }

  private void attemptRotorSeedFromCANCoders() {
    AbsSensorRead absRead = readAbsSensors();
    if (!absRead.ok()) {
      if (!"NO_DEVICES".equals(absRead.status())) {
        SmartDashboard.putString("Turret/CRT/SeedStatus", absRead.status());
      }
      lastSeedStatus = absRead.status();
      return;
    }

    double absA = absRead.absA();
    double absB = absRead.absB();
    lastAbsA = absA;
    lastAbsB = absB;

    var solver = new EasyCRT(easyCrtConfig);
    var solvedAngle = solver.getAngleOptional();

    SmartDashboard.putNumber("Turret/CRT/AbsA", absA);
    SmartDashboard.putNumber("Turret/CRT/AbsB", absB);
    SmartDashboard.putString("Turret/CRT/SolverStatus", solver.getLastStatus().toString());
    SmartDashboard.putNumber("Turret/CRT/SolverErrorRot", solver.getLastErrorRotations());
    SmartDashboard.putNumber("Turret/CRT/SolverIterations", solver.getLastIterations());

    if (solvedAngle.isEmpty()) {
      SmartDashboard.putBoolean("Turret/CRT/SolutionFound", false);
      lastSeedStatus = solver.getLastStatus().toString();
      return;
    }

    Angle turretRotations = solvedAngle.get();
    smartMotorController.setEncoderPosition(turretRotations);
    rotorSeededFromAbs = true;
    lastSeededTurretDeg = turretRotations.in(Degrees);
    lastSeedError = solver.getLastErrorRotations();
    SmartDashboard.putBoolean("Turret/CRT/SolutionFound", true);
    SmartDashboard.putNumber("Turret/CRT/SeededTurretDeg", lastSeededTurretDeg);
    SmartDashboard.putNumber("Turret/CRT/MatchErrorRot", lastSeedError);

    lastSeedStatus = "OK";
    SmartDashboard.putString("Turret/CRT/SeedStatus", lastSeedStatus);
    SmartDashboard.putBoolean("Turret/CRT/Seeded", rotorSeededFromAbs);
  }

  private void logCrtConfigTelemetry() {
    double mechanismRangeRot = easyCrtConfig.getMechanismRange().in(Rotations);
    double uniqueCoverageRot = easyCrtConfig.getUniqueCoverage().map(angle -> angle.in(Rotations)).orElse(Double.NaN);
    SmartDashboard.putNumber(
        "Turret/CRT/Config/RatioA", easyCrtConfig.getEncoder1RotationsPerMechanismRotation());
    SmartDashboard.putNumber(
        "Turret/CRT/Config/RatioB", easyCrtConfig.getEncoder2RotationsPerMechanismRotation());
    SmartDashboard.putNumber("Turret/CRT/Config/UniqueCoverageRot", uniqueCoverageRot);
    SmartDashboard.putBoolean(
        "Turret/CRT/Config/CoverageSatisfiesRange", easyCrtConfig.coverageSatisfiesRange());
    SmartDashboard.putNumber("Turret/CRT/Config/RequiredRangeRot", mechanismRangeRot);

    var configPair = easyCrtConfig.getRecommendedCrtGearPair();
    SmartDashboard.putBoolean("Turret/CRT/Config/RecommendedPairFound", configPair.isPresent());
    if (configPair.isPresent()) {
      var pair = configPair.get();
      SmartDashboard.putNumber("Turret/CRT/Config/Reccomender/RecommendedGearA", pair.gearA());
      SmartDashboard.putNumber("Turret/CRT/Config/Reccomender/RecommendedGearB", pair.gearB());
      SmartDashboard.putNumber(
          "Turret/CRT/Config/Reccomender/RecommendedCoverageRot", pair.coverage().in(Rotations));
      SmartDashboard.putNumber("Turret/CRT/Config/Reccomender/RecommendedLcm", pair.lcm());
      SmartDashboard.putBoolean(
          "Turret/CRT/Config/Reccomender/RecommendedCoprime",
          EasyCRTConfig.isCoprime(pair.gearA(), pair.gearB()));
      SmartDashboard.putNumber(
          "Turret/CRT/Config/Reccomender/RecommendedIterations", pair.theoreticalIterations());
    }
  }

  private static record AbsSensorRead(boolean ok, double absA, double absB, String status) {
  }

  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    if (pivot != null) {
      pivot.simIterate();
    }
  }
}
