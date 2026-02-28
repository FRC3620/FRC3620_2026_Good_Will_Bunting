package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Amp;
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
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import org.usfirst.frc3620.CANDeviceType;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
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
  private TalonFX motor = null;
  private SmartMotorController motorController = null;
  private Elevator elevator = null;

  boolean isCalibrated = false;
  boolean activeCalibrating = false;
  private Command calibrationCommand;

  private final Voltage CALIBRATION_VOLTAGE = Volts.of(-1);
  private final LinearVelocity VELOCITY_THRESHOLD = MetersPerSecond.of(.03);
  private final Current CURRENT_THRESHOLD = Amps.of(10);
  private final Time STALL_TIME_SECONDS = Seconds.of(.1);

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
    boolean makeDevice = RobotContainer.canDeviceFinder.isDevicePresent(CANDeviceType.TALON_PHOENIX6, motorId, telemetryPrefix)
        || RobotContainer.shouldMakeAllCANDevices();
    if (makeDevice) {
      motor = new TalonFX(motorId);
      RobotContainer.healthSubsystem.addMotorToWatch(motor, telemetryPrefix, HealthSubsystem.healthOptionsForYAMS);

      SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
          .withMechanismCircumference(Inches.of(2.75).times(Math.PI))
          .withClosedLoopController(100, 0, 0, MetersPerSecond.of(0.2), MetersPerSecondPerSecond.of(0.2))
          .withFeedforward(new ArmFeedforward(0.0, 0, 0, 0))
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(45)))
          .withIdleMode(MotorMode.BRAKE)
          .withTelemetry(telemetryPrefix + "Motor", TelemetryVerbosity.HIGH)
          .withStatorCurrentLimit(Amps.of(40))
          .withControlMode(ControlMode.CLOSED_LOOP);

      motorController = new TalonFXWrapper(motor, DCMotor.getKrakenX60(1), motorConfig);

      createElevator(CALIBRATED_POS);

      setDefaultCommand(idle());

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

      /*if (!activeCalibrating && !isCalibrated) {
        calibrationCommand = calibrate();
        CommandScheduler.getInstance().schedule(calibrationCommand);
      } */

      elevator.updateTelemetry();
      elevator.getMechanismSetpoint().ifPresent(setpoint -> SmartDashboard.putNumber(
          "frc3620/" + telemetryPrefix + "/setPos",
          setpoint.in(Degrees)));
      SmartDashboard.putNumber("frc3620/" + telemetryPrefix + "/actualPosMeters", getExtension().in(Meters));

      LinearVelocity velocity = motorController.getMeasurementVelocity();
      SmartDashboard.putNumber("frc3620/" + telemetryPrefix + "/measurementVelocity", velocity.in(MetersPerSecond));
      SmartDashboard.putNumber("frc3620/" + telemetryPrefix + "/motorVoltage", motorController.getVoltage().in(Volts));
      SmartDashboard.putBoolean("frc3620/" + telemetryPrefix + "/isCalibrated", isCalibrated);

    }
  }

  @Override
  public void simulationPeriodic() {
    if (elevator != null) {
      elevator.simIterate();
    }
  }

  public Command createSetExtensionCommand(Supplier<Distance> distance) {
    Command rv;
    if (elevator != null) {
      rv = elevator.run(distance.get());
    } else {
      rv = idle();
    }
    return rv.withName(telemetryPrefix + " SetExtension");
  }

  public Command setExtensionDashboardCommand() {
    if (elevator != null) {
        return createSetExtensionCommand(() -> Meters.of(SmartDashboard.getNumber("frc3620/" + telemetryPrefix + "/setExtenstionDashboard", 0)));
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
      private Time stallStartTime = Seconds.of(-1);

      {
        addRequirements(IntakeShoulderSubsystem.this);
      }

      public void initialize() {
        motorController.stopClosedLoopController();
        isCalibrated = false;
        activeCalibrating = true;
        stallStartTime = Seconds.of(-1);
      }

      public void execute() {

        motorController.setVoltage(CALIBRATION_VOLTAGE);

        LinearVelocity velocity = motorController.getMeasurementVelocity();
        Current current = motorController.getStatorCurrent();

        if (Math.abs(velocity.in(MetersPerSecond)) < VELOCITY_THRESHOLD.in(MetersPerSecond) && current.gte(Amps.of(10))) {
          if (stallStartTime.lt(Seconds.zero())) {
            stallStartTime = Seconds.of(Timer.getFPGATimestamp());
          }
        } else {
          stallStartTime = Seconds.of(-1);
        }
      }

      public boolean isFinished() {
        if (stallStartTime.lt(Seconds.zero()))
          return false;

        return (Seconds.of(Timer.getFPGATimestamp()).minus(stallStartTime)).gte(STALL_TIME_SECONDS);
      }

      public void end(boolean interrupted) {
        motorController.setVoltage(Volts.zero());
        activeCalibrating = false;

        if (!interrupted) {
          motorController.setEncoderPosition(CALIBRATED_POS);

          motorController.startClosedLoopController();
          motorController.setPosition(CALIBRATED_POS);
  
          createElevator(CALIBRATED_POS);
    
          isCalibrated = true;

        }
      }
    }
        .withName("Intake Shoulder Calibration");
  }

}
