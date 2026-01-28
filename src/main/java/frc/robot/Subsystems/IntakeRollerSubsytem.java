package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Pound;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import java.util.function.Supplier;

import org.usfirst.frc3620.CANDeviceType;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

public class IntakeRollerSubsytem extends SubsystemBase {
    int motorId = 57;
    boolean forceNullHardwareForTesting = false;  // set this to true to test the "we don't have hardware" configuration

    private TalonFX rollers = null;
    private SmartMotorController rollerMotor = null;
    private FlyWheel roller = null;

    public IntakeRollerSubsytem() {
        boolean makeDevices = RobotContainer.canDeviceFinder.isDevicePresent(CANDeviceType.TALON_PHOENIX6, motorId,
                        "IntakeRoller") || RobotContainer.shouldMakeAllCANDevices();
        if (forceNullHardwareForTesting) makeDevices = false;

        if (makeDevices) {
            rollers = new TalonFX(motorId);

            // Configure the motor controller FIRST
            SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
                    .withClosedLoopController(
                            0.1, // kP - tune this
                            0.0, // kI
                            0.0, // kD
                            RotationsPerSecond.of(100),
                            RotationsPerSecondPerSecond.of(200))
                    .withGearing(new MechanismGearing(GearBox.fromReductionStages(1, 1))) // Direct drive
                    .withIdleMode(MotorMode.BRAKE)
                    .withTelemetry("RollerMotor", TelemetryVerbosity.HIGH)
                    .withStatorCurrentLimit(Amps.of(40))
                    .withSupplyCurrentLimit(Amps.of(40))
                    .withControlMode(ControlMode.CLOSED_LOOP);

            // Create the motor controller wrapper
            rollerMotor = new TalonFXWrapper(rollers, DCMotor.getKrakenX60(1), motorConfig);

            // NOW create the FlyWheel config WITH the motor controller
            FlyWheelConfig rollerConfig = new FlyWheelConfig(rollerMotor)
                    .withDiameter(Inch.of(4))
                    .withMass(Pound.of(0.5))
                    .withUpperSoftLimit(RPM.of(2000))
                    .withTelemetry("Intake Roller", TelemetryVerbosity.HIGH);

            // Create the FlyWheel
            roller = new FlyWheel(rollerConfig);
        }
    }

    private Command doNothingCommand() {
        return run(() -> {});
    }

    public Command setSpeedCommand(Supplier<AngularVelocity> angularVelocitySupplier) {
        if (rollers == null) {
            return doNothingCommand();
        } else {
            return roller.setSpeed(angularVelocitySupplier);
        }
    }

    public Command setSpeedCommand(AngularVelocity angularVelocity) {
        if (rollers == null) {
            return doNothingCommand();
        } else {
            return roller.setSpeed(angularVelocity);
        }
    }

    public Command rollersOnCommand() {
        // Only use YAMS control, not manual rollers.set()
        return setSpeedCommand(RPM.of(1500)).withName("Rollers On");
    }

    public Command rollersOffCommand() {
        return setSpeedCommand(RPM.of(0)).withName("Rollers Off");
    }

    public Command rollersBackwardsCommand() {
        return setSpeedCommand(RPM.of(-1500)).withName("Rollers Backwards");
    }

    @Override
    public void periodic() {
        if (roller != null) {
            roller.updateTelemetry();
        }
    }

    @Override
    public void simulationPeriodic() {
        if (roller != null) {
            roller.simIterate();
        }
    }
}