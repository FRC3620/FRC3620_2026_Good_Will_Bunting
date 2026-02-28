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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
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

  boolean isCalibrated = false;
  boolean activeCalibrating = false;
  private Command calibrationCommand;

  private final Voltage CALIBRATION_VOLTAGE = Volts.of(-.5);
  private final double VELOCITY_THRESHOLD = 2.0;
  private final Current CURRENT_THRESHOLD = Amps.of(10);
  private final double STALL_TIME_SECONDS = .25;

  private final Angle CALIBRATED_POS = Degrees.of(0.0); // place holders

  public enum IntakeShoulderPositions {
    Out(Degrees.of(120)),
    IN(Degree.of(0));

    private final Angle angle;;

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
          .withClosedLoopController(4.0, 0, 0, DegreesPerSecond.of(100), DegreesPerSecondPerSecond.of(100))
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(27 / 1, 24 / 15)))
          .withIdleMode(MotorMode.BRAKE)
          .withTelemetry(telemetryPrefix + "Motor", TelemetryVerbosity.HIGH)
          .withStatorCurrentLimit(Amps.of(20))
          .withControlMode(ControlMode.CLOSED_LOOP);

      motorController = new TalonFXWrapper(motor, DCMotor.getKrakenX60(1), motorConfig);

      createPivot(CALIBRATED_POS);

      setDefaultCommand(idle());

    }
    SmartDashboard.putNumber("frc3620/" + telemetryPrefix + "/setExtenstionDashboard", 0);
  }

  private void createPivot(Angle startingAngle) {
    pivot = new Pivot(new PivotConfig(motorController)
        // Starting position of the Pivot
        .withStartingPosition(IntakeShoulderPositions.IN.angle)
        // .withWrapping(Degrees.of(0), Degrees.of(360))
        // Hard limit bc wiring prevents infinite spinning
        .withSoftLimits(IntakeShoulderPositions.IN.angle, IntakeShoulderPositions.Out.angle)
        // Telemetry
        .withTelemetry(telemetryPrefix, TelemetryVerbosity.HIGH));
  }

  @Override
  public void periodic() {
    if (pivot != null) {

      /*
       * if (!activeCalibrating && !isCalibrated) {
       * calibrationCommand = calibrate();
       * CommandScheduler.getInstance().schedule(calibrationCommand);
       * }
       */

      pivot.updateTelemetry();
      pivot.getMechanismSetpoint().ifPresent(setpoint -> SmartDashboard.putNumber(
          "frc3620/" + telemetryPrefix + "/setPos",
          setpoint.in(Degrees)));
      SmartDashboard.putNumber("frc3620/" + telemetryPrefix + "/actualPosDegrees", getPosition().in(Degrees));
      SmartDashboard.putNumber("frc3620/" + telemetryPrefix + "/rotorVelocity",
          motorController.getMechanismVelocity().in(DegreesPerSecond));
      SmartDashboard.putNumber("frc3620/" + telemetryPrefix + "/motorVoltage", motorController.getVoltage().in(Volts));
      SmartDashboard.putBoolean("frc3620/" + telemetryPrefix + "/isCalibrated", isCalibrated);

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
      rv = pivot.run(angle.get());
    } else {
      rv = idle();
    }
    return rv.withName(telemetryPrefix + " SetPosition");
  }

  public Command setPositionDashboardCommand() {
    if (pivot != null) {
      return createSetPositionCommand(
          () -> Degrees.of(SmartDashboard.getNumber("frc3620/" + telemetryPrefix + "/setPositionDashboard", 0)));
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

        if (Math.abs(velocity) < VELOCITY_THRESHOLD && current > 10) {
          if (stallStartTime < 0) {
            stallStartTime = Timer.getFPGATimestamp();
          }
        } else {
          stallStartTime = -1;
        }

        SmartDashboard.putNumber("frc3620/IntakeShoulder/Timer", Timer.getFPGATimestamp() - stallStartTime);
      }

      public boolean isFinished() {
        if (stallStartTime < 0)
          return false;

        SmartDashboard.putBoolean("frc3620/IntakeShoulder/ShouldFinish",
            (Timer.getFPGATimestamp() - stallStartTime) > STALL_TIME_SECONDS);
        return (Timer.getFPGATimestamp() - stallStartTime) > STALL_TIME_SECONDS;
      }

      public void end(boolean interrupted) {
        motorController.setDutyCycle(0);
        activeCalibrating = false;

        SmartDashboard.putBoolean("INTAKE SHOULDER END RAN", true);

        if (!interrupted) {
          motorController.setEncoderPosition(CALIBRATED_POS);
          motorController.startClosedLoopController();

          motorController.setPosition(CALIBRATED_POS);

          isCalibrated = true;

        }

      }
    }
        .withName("Intake Shoulder Calibration");
  }

}
