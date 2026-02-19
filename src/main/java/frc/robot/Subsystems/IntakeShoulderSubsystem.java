package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Amps;
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
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import org.usfirst.frc3620.CANDeviceType;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
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
import yams.mechanisms.positional.Arm;
import yams.mechanisms.positional.Elevator;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class IntakeShoulderSubsystem extends SubsystemBase {
  int motorId = Constants.MOTORID_INTAKE_SHOULDER;
  String telemetryPrefix = "IntakeShoulder";
  private Distance setpoint = Meters.of(0);
  private TalonFX motor = null;
  private SmartMotorController motorController = null;
  private Elevator elevator = null;

  boolean isCalibrated = false;
  boolean activeCalibrating = false;

  private final Voltage CALIBRATION_VOLTAGE = Volts.of(-.5);
  private final double VELOCITY_THRESHOLD = 10.0; // deg/sec
  private final double STALL_TIME_SECONDS = .1;

  private final Distance CALIBRATED_POS = Meters.of(0.0); // place holders

  public enum IntakeShoulderPositions {
    Out(Meters.of(0.35)),
    IN(Meters.of(0.0));

    private final Distance distance;

    IntakeShoulderPositions(Distance i) {
      this.distance = i;
    }

    public Distance getDistance() {
      return distance;
    }
  }

  public IntakeShoulderSubsystem() {
    boolean makeDevice = RobotContainer.canDeviceFinder.isDevicePresent(CANDeviceType.TALON_PHOENIX6, motorId)
        || RobotContainer.shouldMakeAllCANDevices();
    if (makeDevice) {
      motor = new TalonFX(motorId);
      RobotContainer.healthSubsystem.addMotorToWatch(motor, telemetryPrefix, HealthSubsystem.healthOptionsForYAMS);

      SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
          .withMechanismCircumference(Inches.of(3.5).times(Math.PI))
          .withClosedLoopController(100, 0, 0, MetersPerSecond.of(0.2), MetersPerSecondPerSecond.of(0.2))
          .withFeedforward(new ArmFeedforward(0.0, 0, 0, 0))
          .withGearing(new MechanismGearing(GearBox.fromTeeth(24, 36)))
          .withIdleMode(MotorMode.BRAKE)
          .withTelemetry(telemetryPrefix + "Motor", TelemetryVerbosity.HIGH)
          .withStatorCurrentLimit(Amps.of(40))
          .withControlMode(ControlMode.CLOSED_LOOP);

      motorController = new TalonFXWrapper(motor, DCMotor.getKrakenX60(1), motorConfig);

      createElevator(CALIBRATED_POS);

      setDefaultCommand(elevator.setHeight(() -> setpoint));

    }
    SmartDashboard.putNumber("frc3620/" + telemetryPrefix + "/setExtenstionDashboard", 0);
  }

  private void createElevator(Distance startingHeight) {
    elevator = new Elevator(new ElevatorConfig(motorController)
        .withHardLimits(IntakeShoulderPositions.IN.getDistance(), IntakeShoulderPositions.Out.getDistance())
        .withStartingHeight(startingHeight)
        .withTelemetry(telemetryPrefix, TelemetryVerbosity.HIGH)
        .withMass(Pound.of(5)));
  }

  @Override
  public void periodic() {
    if (elevator != null) {

      if (!isCalibrated && !activeCalibrating) {
        CommandScheduler.getInstance().schedule(calibrate());
      }

      elevator.updateTelemetry();
      elevator.getMechanismSetpoint().ifPresent(setpoint -> SmartDashboard.putNumber(
          "frc3620/" + telemetryPrefix + "/setPos",
          setpoint.in(Degrees)));
      SmartDashboard.putNumber("frc3620/" + telemetryPrefix + "/setpoint meters", setpoint.in(Meters));
      SmartDashboard.putNumber("frc3620/" + telemetryPrefix + "/actualPosMeters", getExtension().in(Meters));
    }
  }

  @Override
  public void simulationPeriodic() {
    if (elevator != null) {
      elevator.simIterate();
    }
  }

  public Command setExtension(Supplier<Distance> distance) {
    Command rv;
    if (elevator != null) {
      rv = run(() -> {
        setpoint = distance.get();
      });
    } else {
      rv = idle();
    }
    return rv.withName(telemetryPrefix + " SetExtension");
  }

  public Command setExtensionDashboardCommand() {
    if (elevator != null) {

      return run(() -> {
        setpoint = Meters.of(SmartDashboard.getNumber("frc3620/" + telemetryPrefix + "/setExtenstionDashboard", 0));
      });
    } else {
      return idle();
    }
  }

  public Distance getExtension() {
    if (elevator == null)
      return Meters.of(999999);
    else {
      return elevator.getHeight();
    }
  }

  public Command calibrate() {
    return new Command() {
      private double stallStartTime = -1;

      public void initialize() {
        isCalibrated = false;
        activeCalibrating = true;
        stallStartTime = -1;
      }

      public void execute() {
        motorController.setVoltage(CALIBRATION_VOLTAGE);

        double velocity = motorController.getMechanismVelocity().in(DegreesPerSecond);
        double current = motorController.getStatorCurrent().in(Amps);

        if (Math.abs(velocity) < VELOCITY_THRESHOLD && current > 10) {
          if (stallStartTime < 0) {
            stallStartTime = Timer.getFPGATimestamp();
          }
        } else {
          stallStartTime = -1;
        }
      }

      public boolean isFinished() {
        if (stallStartTime < 0)
          return false;

        return Timer.getFPGATimestamp() - stallStartTime > STALL_TIME_SECONDS;
      }

      public void end(boolean interrupted) {
        motorController.setVoltage(Volts.zero());

        motorController.setPosition(CALIBRATED_POS);

        createElevator(CALIBRATED_POS);
        isCalibrated = true;
        activeCalibrating = false;
      }
    }
        .withName("Intake Shoulder Calibration");
  }

}
