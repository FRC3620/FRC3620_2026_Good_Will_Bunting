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

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.usfirst.frc3620.CANDeviceType;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
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

  Timer atTargetTime = new Timer();

  /** Manually rerun CRT seeding. */
  private static final String RERUN_SEED = "Turret/CRT/RerunSeed";

  private SmartMotorController smartMotorController = null;
  private Pivot pivot = null;

  private static final Angle absAEncoderOffset = Rotations.of(-0.669189453125);
  private static final Angle absBEncoderOffset = Rotations.of(-0.955078125);

  private SlewRateLimiter turretLimiter = new SlewRateLimiter(180.0);
  private Angle filteredTargetAngle = Degrees.of(0);
  private double turretFilterAlpha = 1; // smoothing factor
  private double turretTargetingOffset = 0;

  private boolean atTarget = false;

  private Angle nearRightWrappingAngle = Degrees.of(-293 + 15);
  private Angle reallyCloseToRightWrappingAngle = Degrees.of(-293 + 7);
  private Angle nearLeftWrappingAngle = Degrees.of(135 - 15);
  private Angle reallyCloseTOLeftWrappingAngle = Degrees.of(135 - 7);

  private Angle targetAngle = Degrees.of(0);

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
          .withClosedLoopController(110, 0, 0, DegreesPerSecond.of(2500), DegreesPerSecondPerSecond.of(2500))
          // Configure Motor and Mechanism properties
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(50.0 / 14.0, 140.0 / 18.0)))
          // .withContinuousWrapping(Degrees.of(0), Degrees.of(360))
          .withIdleMode(MotorMode.BRAKE)
          .withMotorInverted(true)
          // Setup Telemetry
          .withTelemetry("motor", TelemetryVerbosity.LOW)
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
          // .withWrapping(Degrees.of(0), Degrees.of(360))
          // Hard limit bc wiring prevents infinite spinning
          .withHardLimit(Degrees.of(-298), Degrees.of(135))
          .withSoftLimits(Degrees.of(-298), Degrees.of(135))
          // Telemetry
          .withTelemetry(telemetryPrefix, TelemetryVerbosity.LOW)
          // MOI Calculation
          .withMOI(Meters.of(0.25), Pounds.of(2)));

      setDefaultCommand(idle());
    }

    easyCrtConfig = buildEasyCRTConfig();
    logCrtConfigTelemetry();
    SmartDashboard.putBoolean(RERUN_SEED, false);

    SmartDashboard.putNumber("frc3620/" + telemetryPrefix + "/Angle Dashboard Control", 180);
    SmartDashboard.putNumber("frc3620/" + telemetryPrefix + "/Filtering Alpha", turretFilterAlpha);
    SmartDashboard.putNumber("frc3620/" + telemetryPrefix + "/Targeting Offset Degrees", turretTargetingOffset);
    SmartDashboard.putNumber("frc3620/ShotCalculator/XVelocityMultipler", 1);
    SmartDashboard.putNumber("frc3620/ShotCalculator/YVelocityMultipler", 1);
  }

  public Command createSetAngleCommand(Supplier<Angle> angle) {
    Command rv;
    Supplier<Angle> setpt = angle;
    if (pivot == null) {
      rv = idle();
    } else {
      // setpt = () -> Degrees.of(MathUtil.inputModulus(angle.get().in(Degrees), -232,
      // 128));
      setpt = () -> closestAngle(angle);
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
      rv = createSetAngleCommand(
          () -> Degrees.of(SmartDashboard.getNumber("frc3620/" + telemetryPrefix + "/Angle Dashboard Control", 180)));
    }
    return rv.withName(telemetryPrefix + " setAngleDashboard");
  }

  public Command createSetAngleToTargetCommand(Translation2d targetPosition, Supplier<Pose2d> robotPose,
      Supplier<VelocityVector> robotVelocity) {
    Command rv;
    if (pivot == null) {
      rv = idle();
    } else {
      filteredTargetAngle = getAngle(); // Initialize filtered angle to current angle
      rv = createSetAngleCommand(
          () -> {
            Angle raw = ShotCalculator.calculateNetTurretAngleToTarget(targetPosition, robotPose, robotVelocity);
            double alpha = SmartDashboard.getNumber("frc3620/" + telemetryPrefix + "/Filtering Alpha",
                turretFilterAlpha);
            turretFilterAlpha = MathUtil.clamp(alpha, 0.0, 1.0);
            filteredTargetAngle = filteredTargetAngle.times(1.0 - alpha).plus(raw.times(alpha));
            Angle targetingOffsetAngle = Degrees.of(SmartDashboard
                .getNumber("frc3620/" + telemetryPrefix + "/Targeting Offset Degrees", turretTargetingOffset));
            targetAngle = filteredTargetAngle.plus(targetingOffsetAngle);
            return targetAngle;
          });
    }
    return rv.withName(telemetryPrefix + " setAngleToTarget");
  }

  public static Angle closestAngle(Supplier<Angle> target) {

    Angle targetA = target.get();

    if (targetA.gte(Degrees.of(135))) {
      return targetA.minus(Degrees.of(360));
    } else if (targetA.lte(Degrees.of(-298))) {
      return targetA.plus(Degrees.of(360));
    }
    return targetA;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (!startTimer) {
      startUpTimer.reset();
      startUpTimer.start();
      startTimer = true;
    }

    SmartDashboard.putNumber("frc3620/" + telemetryPrefix + "/AtTargetTimer", atTargetTime.get());

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
      SmartDashboard.putBoolean("frc3620/" + telemetryPrefix + "/AtTarget", atTarget().getAsBoolean());
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
        .withAbsoluteEncoderInversions(false, false)
        .withMechanismRange(
            // Rotations.of(-225/360), Rotations.of(180/360))
            Rotations.of(-1), Rotations.of(1))
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

  public boolean isNearRightWrapping() {
  return getAngle().isNear(nearRightWrappingAngle, 5);
}

public boolean isReallyCloseToRightWrapping() {
  return getAngle().isNear(reallyCloseToRightWrappingAngle, 3);
}

public boolean isNearLeftWrapping() {
  return getAngle().isNear(nearLeftWrappingAngle, 5);
}

public boolean isReallyCloseToLeftWrapping() {
  return getAngle().isNear(reallyCloseTOLeftWrappingAngle, 3);
}

  /*public BooleanSupplier reallyCloseToRightWrapping() {
    Angle currentAngle = getAngle();

    if (currentAngle.isNear(reallyCloseToRightWrappingAngle, 3)) {
      reallyCloseToRightWrapping = true;
    } else {
      reallyCloseToRightWrapping = false;
    }
    return () -> reallyCloseToRightWrapping;
  }

  public BooleanSupplier nearRightWrapping() {
    Angle currentAngle = getAngle();

    if (currentAngle.isNear(nearRightWrappingAngle, 5)) {
      nearRightWrapping = true;
    } else {
      nearRightWrapping = false;
    }
    return () -> nearRightWrapping;
  }

  public BooleanSupplier nearLeftWrapping(){
    Angle currentAngle = getAngle();

    if(currentAngle.isNear(nearLeftWrappingAngle, 5)){
      nearLeftWrapping=true;
    }else{
      nearLeftWrapping = false;
    }
    return ()-> nearLeftWrapping;
  }
  public BooleanSupplier reallyCloseToLeftWrapping(){
    Angle currentAngle = getAngle();

    if(currentAngle.isNear(reallyCloseTOLeftWrappingAngle, 3)){
      reallyCloseToLeftWrapping=true;
    }else{
      reallyCloseToLeftWrapping = false;
    }
    return ()-> reallyCloseToLeftWrapping;
  }*/

  public BooleanSupplier atTarget() {

    atTargetTime.reset();
    Angle current = getAngle();
    if (current.isNear(targetAngle, Degrees.of(10))) {

      atTargetTime.start();

      // if (atTargetTime.hasElapsed(.75)) {
      atTarget = true;
      // }

    } else {
      atTarget = false;
    }

    return () -> atTarget;
  }

  private static record AbsSensorRead(boolean ok, double absA, double absB, String status) {
  }

  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    if (pivot != null) {
      pivot.simIterate();
    }
  }

  public TalonFX getMotor() {
    if (motor != null) {
      return motor;
    }
    return null;
  }
}
