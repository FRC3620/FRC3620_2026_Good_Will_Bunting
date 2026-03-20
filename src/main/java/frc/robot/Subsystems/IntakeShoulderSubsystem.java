package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Pound;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.Set;
import java.util.function.Supplier;

import org.usfirst.frc3620.CANDeviceType;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.IntakeShoulderSubsystem.IntakeShoulderPositions;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Arm;
import yams.mechanisms.positional.Elevator;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class IntakeShoulderSubsystem extends SubsystemBase {
  int motorId = Constants.MOTORID_INTAKE_SHOULDER;
  String telemetryPrefix = "IntakeShoulder";
  private TalonFX motor = null;
  private SmartMotorController motorController = null;
  private Pivot pivot = null;

  Command calibrationCommand;
  boolean isCalibrated = false;
  boolean activeCalibrating = false;

  private final Voltage CALIBRATION_VOLTAGE = Volts.of(-2);
  private final double VELOCITY_THRESHOLD = 5.0;
  private final Current CURRENT_THRESHOLD = Amps.of(3);
  private final double STALL_TIME_SECONDS = .25;

  private final Angle CALIBRATED_POS = Degrees.of(0.0); // place holders

  public enum IntakeShoulderPositions {
    OUT(Degrees.of(100)),
    IN(Degrees.of(0)),
    JOSTLE_TOP(Degrees.of(35)),
    JOSTLE_BOTTOM(Degrees.of(50));

    private final Angle angle;

    IntakeShoulderPositions(Angle a) {
      this.angle = a;
    }

    public Angle getAngle() {
      return angle;
    }
  }

  public IntakeShoulderSubsystem() {
    boolean makeDevice = RobotContainer.canDeviceFinder.isDevicePresent(CANDeviceType.TALON_PHOENIX6, motorId,
        telemetryPrefix)
        || RobotContainer.shouldMakeAllCANDevices();
    if (makeDevice) {
      motor = new TalonFX(motorId);
      RobotContainer.healthSubsystem.addMotorToWatch(motor, telemetryPrefix, HealthSubsystem.healthOptionsForYAMS);

      SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
          .withClosedLoopController(200.0, 0, 0, DegreesPerSecond.of(720), DegreesPerSecondPerSecond.of(720))
          .withFeedforward(new ArmFeedforward(0, 0.5, 0))
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(27.0 / 1.0, 34.0 / 22.0)))
          .withIdleMode(MotorMode.COAST)
          .withTelemetry(telemetryPrefix + "Motor", TelemetryVerbosity.LOW)
          .withStatorCurrentLimit(Amps.of(40))
          .withControlMode(ControlMode.CLOSED_LOOP);

      motorController = new TalonFXWrapper(motor, DCMotor.getKrakenX60(1), motorConfig);

      createPivot(CALIBRATED_POS);

    }
    SmartDashboard.putNumber("frc3620/" + telemetryPrefix + "/setExtenstionDashboard", 0);
    SmartDashboard.putData(this);

  }

  private void createPivot(Angle startingAngle) {

    Angle softLimit = Degrees.of(-150.0);

    if (isCalibrated) {
      softLimit = Degrees.of(0.0);
    }

    pivot = new Pivot(new PivotConfig(motorController)
        // Starting position of the Pivot
        .withStartingPosition(IntakeShoulderPositions.IN.angle)
        // .withWrapping(Degrees.of(0), Degrees.of(360))
        // Hard limit bc wiring prevents infinite spinning
        .withSoftLimits(softLimit, softLimit.plus(Degrees.of(369)))
        .withHardLimit(softLimit, softLimit.plus(Degrees.of(369)))
        .withMOI(Inches.of(25), Pound.of(1))
        // Telemetry
        .withTelemetry(telemetryPrefix, TelemetryVerbosity.LOW));
  }

  @Override
  public void periodic() {
    if (pivot != null) {

      if (!activeCalibrating && !isCalibrated && !RobotBase.isSimulation()) {
        calibrationCommand = calibrate();
        CommandScheduler.getInstance().schedule(calibrationCommand);
      }

      pivot.updateTelemetry();
      pivot.getMechanismSetpoint().ifPresent(setpoint -> SmartDashboard.putNumber(
          "frc3620/" + telemetryPrefix + "/setPos",
          setpoint.in(Degrees)));
      SmartDashboard.putNumber("frc3620/" + telemetryPrefix + "/actualPosDegrees", getPosition().in(Degrees));
      SmartDashboard.putNumber("frc3620/" + telemetryPrefix + "/rotorVelocity",
          motorController.getMechanismVelocity().in(DegreesPerSecond));
      SmartDashboard.putNumber("frc3620/" + telemetryPrefix + "/motorVoltage", motorController.getVoltage().in(Volts));
      SmartDashboard.putBoolean("frc3620/" + telemetryPrefix + "/isCalibrated", isCalibrated);
      SmartDashboard.putBoolean("frc3620/" + telemetryPrefix + "/activeCalibration", activeCalibrating);
      SmartDashboard.putNumber("frc3620/" + telemetryPrefix + "/motorCurrent",
          motorController.getStatorCurrent().in(Amps));
      SmartDashboard.putData("frc3620/" + telemetryPrefix + "/subsystem", this);

    }
  }

  @Override
  public void simulationPeriodic() {
    if (pivot != null) {
      pivot.simIterate();
    }
  }

  public Command createSetPositionCommand(Supplier<Angle> angle) {
    Command rv;
    if (pivot != null) {
      rv = pivot.setAngle(angle);
    } else {
      rv = idle();
    }
    return rv.withName(telemetryPrefix + " SetPosition");
  }

  public Command createSetPositionCommandGated(Supplier<Angle> angle) {
    Command rv;
    if (pivot != null) {
      rv = Commands.waitUntil(() -> isCalibrated).andThen(Commands.none())
          .andThen(createSetPositionCommand(angle).asProxy());
    } else {
      rv = idle();
    }
    return rv.withName(telemetryPrefix + " SetPosition");
  }

  public Command createSetPositionThenCoast(Supplier<Angle> angle) {
    if (pivot != null) {
      return pivot.setAngle(angle)
          .onlyWhile(() -> Math.abs(pivot.getAngle().in(Degrees) - angle.get().in(Degrees)) > 20);
    }
    return idle();
  }

  public Command createJostleCommand() {

    if (pivot == null) {
      return idle();
    }
    Time UP_HOLD_TIME = Seconds.of(0.5);
    Time DOWN_HOLD_TIME = Seconds.of(0.5);

    return Commands.sequence(
        createSetPositionCommandGated(() -> IntakeShoulderPositions.JOSTLE_BOTTOM.getAngle()).withTimeout(DOWN_HOLD_TIME),
        createSetPositionCommandGated(() -> IntakeShoulderPositions.JOSTLE_TOP.getAngle()).withTimeout(UP_HOLD_TIME))
        .repeatedly().withName("Jostle Command");
  }

  public Command setPositionDashboardCommand() {
    if (pivot != null) {
      return createSetPositionCommand(
          () -> Degrees.of(SmartDashboard.getNumber("frc3620/" + telemetryPrefix + "/setExtenstionDashboard", 0)));
    } else {
      return idle();
    }
  }

  public Angle getPosition() {
    if (pivot == null)
      return Degrees.of(999999);
    else {
      return pivot.getAngle();
    }
  }

  public Command createDoNothingCommand() {
    if (pivot != null) {
      return idle();
    }
    return idle();
  }

  public Command calibrate() {
    return new Command() {

      {
        addRequirements(IntakeShoulderSubsystem.this);
      }

      private double stallStartTime = -1;

      public void initialize() {
        // setDefaultCommand(idle());
        activeCalibrating = true;
        isCalibrated = false;
        stallStartTime = -1;
        motorController.stopClosedLoopController();
      }

      public void execute() {
        motorController.setVoltage(CALIBRATION_VOLTAGE);
        // motorController.setDutyCycle(-0.2);

        double velocity = motorController.getMechanismVelocity().in(DegreesPerSecond);
        double current = motorController.getStatorCurrent().in(Amps);

        if (Math.abs(velocity) < VELOCITY_THRESHOLD && current > CURRENT_THRESHOLD.in(Amps)) {
          if (stallStartTime < 0) {
            stallStartTime = Timer.getFPGATimestamp();
          }
        } else {
          stallStartTime = -1;
        }

        SmartDashboard.putNumber("frc3620/IntakeShoulder/Timer", Timer.getFPGATimestamp() - stallStartTime);
        SmartDashboard.putNumber("frc3620/IntakeShoulder/StallStart", stallStartTime);
      }

      public boolean isFinished() {
        SmartDashboard.putBoolean("frc3620/IntakeShoulder/ShouldFinish",
            stallStartTime > STALL_TIME_SECONDS);
        if (stallStartTime < 0)
          return false;

        return (Timer.getFPGATimestamp() - stallStartTime) > STALL_TIME_SECONDS;
      }

      public void end(boolean interrupted) {
        motorController.setDutyCycle(0);
        activeCalibrating = false;

        SmartDashboard.putBoolean("INTAKE SHOULDER END RAN", true);

        if (!interrupted) {
          createPivot(CALIBRATED_POS);
          motorController.setEncoderPosition(CALIBRATED_POS);
          motorController.startClosedLoopController();

          motorController.setPosition(CALIBRATED_POS);

          isCalibrated = true;

        }

      }
    }
        .withName("Intake Shoulder Calibration");
  }

    public TalonFX getMotor() {
        if (motor != null) {
            return motor;
        }
        return null;
    }

}
