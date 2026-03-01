package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Pound;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.hal.CANAPITypes.CANDeviceType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
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
import org.usfirst.frc3620.CANDeviceFinder;

public class ConveyerSubsystem extends SubsystemBase {
    int motorId = Constants.MOTORID_CONVEYER;
    String telemetryPrefix = "Conveyer";

    private TalonFX motor = null;
    private SmartMotorController motorController;
    private FlyWheel flyWheel;

    public ConveyerSubsystem() {
        boolean makeDevices = RobotContainer.canDeviceFinder.isDevicePresent(
                org.usfirst.frc3620.CANDeviceType.TALON_PHOENIX6,
                motorId, telemetryPrefix + " Rollers") || RobotContainer.shouldMakeAllCANDevices();

        if (makeDevices) {
            motor = new TalonFX(motorId);
            RobotContainer.healthSubsystem.addMotorToWatch(motor, telemetryPrefix, HealthSubsystem.healthOptionsForYAMS);
            SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
                    .withClosedLoopController(
                            0.1, // kP - tune this
                            0.0, // kI
                            0.0, // kD
                            RotationsPerSecond.of(100),
                            RotationsPerSecondPerSecond.of(200))
                    .withMotorInverted(false)
                    .withGearing(new MechanismGearing(GearBox.fromReductionStages(1, 1))) // Direct drive
                    .withIdleMode(MotorMode.BRAKE)
                    .withTelemetry(telemetryPrefix + " Motor", TelemetryVerbosity.HIGH)
                    .withStatorCurrentLimit(Amps.of(40))
                    .withSupplyCurrentLimit(Amps.of(40))
                    .withControlMode(ControlMode.CLOSED_LOOP);

            motorController = new TalonFXWrapper(motor, DCMotor.getKrakenX60(1), motorConfig);
            FlyWheelConfig rollerConfig = new FlyWheelConfig(motorController)
                    .withDiameter(Inch.of(4))
                    .withMass(Pound.of(0.5))
                    .withUpperSoftLimit(RPM.of(7000))
                    .withTelemetry(telemetryPrefix + " Roller", TelemetryVerbosity.HIGH);

            // Create the FlyWheel
            flyWheel = new FlyWheel(rollerConfig);

            setDefaultCommand(idle());
        }
        SmartDashboard.putNumber("frc3620/" + telemetryPrefix + "/RPM Dashboard Control", 0);
    }

    public Command setSpeed(Supplier<AngularVelocity> speed) {

        if (flyWheel != null) {
            return flyWheel.setSpeed(speed.get());
        } else {
            return idle();
        }
    }

    @Override
    public void simulationPeriodic() {
        // Only simulate, don't manually run the roller
        flyWheel.simIterate();
    }

    public void periodic() {
        if (flyWheel != null) {
            flyWheel.updateTelemetry();
            SmartDashboard.putNumber("frc3620/" + telemetryPrefix + "/actualSpeedRPM", flyWheel.getSpeed().in(RPM));
        }
    }

    public Command setSpeedDashboardCommand() {
        if (flyWheel != null) {
            return setSpeed(() -> RPM.of(SmartDashboard.getNumber("frc3620/" + telemetryPrefix + "/RPM Dashboard Control", 0)))
            .withName(telemetryPrefix + " Set Speed Dashboard");
        } else {
            return idle();
        }
    }
}
