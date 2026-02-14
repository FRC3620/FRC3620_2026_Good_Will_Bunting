package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pound;

import org.usfirst.frc3620.CANDeviceType;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
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

public class ShooterHoodSubsystem extends SubsystemBase {
    int motorId = Constants.MOTORID_HOOD;
    String telemetryPrefix = "Hood";

    TalonFX motor = null;
    private Pivot pivot = null;
    private SmartMotorController motorController = null;

    public ShooterHoodSubsystem() {

        boolean makeDevices = RobotContainer.canDeviceFinder.isDevicePresent(CANDeviceType.TALON_PHOENIX6,
                motorId, telemetryPrefix) ||
                RobotContainer.shouldMakeAllCANDevices();

        if (makeDevices) {
            motor = new TalonFX(motorId);
            RobotContainer.healthSubsystem.addMotorToWatch(motor, telemetryPrefix, HealthSubsystem.healthOptionsForYAMS);

            SmartMotorControllerConfig motorControllerConfig = new SmartMotorControllerConfig(this)
                    .withClosedLoopController(4, 0, 0, DegreesPerSecond.of(7), DegreesPerSecondPerSecond.of(5))
                    .withSoftLimit(Degrees.of(25), Degrees.of(65))
                    .withMotorInverted(true)
                    .withGearing(new MechanismGearing(GearBox.fromReductionStages(115.625)))
                    .withIdleMode(MotorMode.BRAKE)
                    .withTelemetry("motor", TelemetryVerbosity.HIGH)
                    .withStatorCurrentLimit(Amps.of(15))
                    .withFeedforward(new ArmFeedforward(0, 0, 0, 0))
                    .withMechanismCircumference(Inches.of(20.5).times(Math.PI))
                    .withStartingPosition(Degrees.of(10))
                    .withControlMode(ControlMode.CLOSED_LOOP);
            // .withMOI(Feet.of(4), Pound.of(4));

            motorController = new TalonFXWrapper(motor, DCMotor.getKrakenX60(1), motorControllerConfig);

            pivot = new Pivot(new PivotConfig(motorController)
                    .withHardLimit(Degrees.of(10), Degrees.of(60))
                    .withStartingPosition(Degrees.of(10))
                    .withMOI(Inches.of(55.7), Pound.of(1))
                    .withTelemetry(telemetryPrefix, TelemetryVerbosity.HIGH));
        }

    }

    @Override
    public void periodic() {
        if (pivot != null) {
            pivot.updateTelemetry();
        }
    }

    @Override
    public void simulationPeriodic() {
        if (pivot != null) {
            pivot.simIterate();
        }
    }

    public Command setAngle(Angle angle) {
        Command rv;
        if (pivot != null) {
            rv = pivot.setAngle(angle);
        } else {
            rv = idle();
        }
        return rv.withName(telemetryPrefix + " SetAngle");
    }
}
