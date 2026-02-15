package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pound;

import java.util.function.Supplier;

import org.usfirst.frc3620.CANDeviceType;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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
  private SmartMotorController motorControler = null;
  private Elevator elevator = null;

  public enum IntakeShoulderPositions {
    Out(Meters.of(0.15)),
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
    boolean makeDevice = RobotContainer.canDeviceFinder.isDevicePresent(CANDeviceType.CANCODER_PHOENIX6, motorId)
        || RobotContainer.shouldMakeAllCANDevices();
    if (makeDevice) {
      motor = new TalonFX(motorId);
      RobotContainer.healthSubsystem.addMotorToWatch(motor, telemetryPrefix, HealthSubsystem.healthOptionsForYAMS);

      SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
          .withClosedLoopController(4, 0, 0, DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90))
          .withFeedforward(new ArmFeedforward(0, 0, 0, 0))
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(70)))
          .withMechanismCircumference(Inches.of(1).times(Math.PI))
          .withIdleMode(MotorMode.BRAKE)
          .withTelemetry(telemetryPrefix + "Motor", TelemetryVerbosity.HIGH)
          .withStatorCurrentLimit(Amps.of(40))
          .withControlMode(ControlMode.CLOSED_LOOP);

      motorControler = new TalonFXWrapper(motor, DCMotor.getKrakenX60(1), motorConfig);

      elevator = new Elevator(new ElevatorConfig(motorControler)
          .withHardLimits(IntakeShoulderPositions.IN.getDistance(), IntakeShoulderPositions.Out.getDistance())
          .withStartingHeight(Meters.of(0))
          .withTelemetry(telemetryPrefix, TelemetryVerbosity.HIGH)
          .withMass(Pound.of(5)));

      setDefaultCommand(elevator.setHeight(() -> setpoint));
    }
    SmartDashboard.putNumber("frc3620/" + telemetryPrefix + "/setExtenstionDashboard", 0);
  }

  @Override
  public void periodic() {
    if (elevator != null) {
      elevator.updateTelemetry();
      elevator.getMechanismSetpoint().ifPresent(setpoint -> SmartDashboard.putNumber(
          "frc3620/" + telemetryPrefix + "/setPos",
          setpoint.in(Degrees)));
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
    if(elevator == null) {
      return idle();
    } else {
      return run(() -> {
        setpoint = Meters.of(SmartDashboard.getNumber("frc3620/" + telemetryPrefix + "/setExtenstionDashboard", 0));
      });

    }
  }

  public Distance getExtension() {
    if (elevator == null)
      return Meters.of(999999);
    else {
      return elevator.getHeight();
    }
  }
}
