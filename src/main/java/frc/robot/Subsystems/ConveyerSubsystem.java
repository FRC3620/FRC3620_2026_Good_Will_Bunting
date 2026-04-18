package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Pound;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.hal.CANAPITypes.CANDeviceType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Helpers.ShotCalculator;
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
    private boolean isJammed;
    Timer timeJammed = new Timer();

    public ConveyerSubsystem() {
        boolean makeDevices = RobotContainer.canDeviceFinder.isDevicePresent(
                org.usfirst.frc3620.CANDeviceType.TALON_PHOENIX6,
                motorId, telemetryPrefix + " Rollers") || RobotContainer.shouldMakeAllCANDevices();

        if (makeDevices) {
            motor = new TalonFX(motorId);
            RobotContainer.healthSubsystem.addMotorToWatch(motor, telemetryPrefix,
                    HealthSubsystem.healthOptionsForYAMS);
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
                    .withTelemetry(telemetryPrefix + " Motor", TelemetryVerbosity.LOW)
                    .withStatorCurrentLimit(Amps.of(40))
                    .withSupplyCurrentLimit(Amps.of(40))
                    .withControlMode(ControlMode.CLOSED_LOOP);

            motorController = new TalonFXWrapper(motor, DCMotor.getKrakenX60(1), motorConfig);
            FlyWheelConfig rollerConfig = new FlyWheelConfig(motorController)
                    .withDiameter(Inch.of(4))
                    .withMass(Pound.of(0.5))
                    .withUpperSoftLimit(RPM.of(7000))
                    .withTelemetry(telemetryPrefix + " Roller", TelemetryVerbosity.LOW);

            // Create the FlyWheel
            flyWheel = new FlyWheel(rollerConfig);

            setDefaultCommand(idle());
        }
        SmartDashboard.putNumber("frc3620/" + telemetryPrefix + "/RPM Dashboard Control", 0);
    }

    public Command setSpeed(Supplier<AngularVelocity> speed) {

        if (flyWheel != null) {
            return flyWheel.setSpeed(speed);
        } else {
            return idle();
        }
    }

    @Override
    public void simulationPeriodic() {
        // Only simulate, don't manually run the roller
        if (flyWheel != null) {
            flyWheel.simIterate();
        }
    }

    public void periodic() {
        if (flyWheel != null) {
            flyWheel.updateTelemetry();
            updateJammed();

            SmartDashboard.putNumber("frc3620/" + telemetryPrefix + "/actualSpeedRPM", flyWheel.getSpeed().in(RPM));
            SmartDashboard.putBoolean("frc3620/" + telemetryPrefix + "/isJammed", isJammed);
        }
    }

    public Command setSpeedDashboardCommand() {
        if (flyWheel != null) {
            return setSpeed(
                    () -> RPM.of(SmartDashboard.getNumber("frc3620/" + telemetryPrefix + "/RPM Dashboard Control", 0)))
                    .withName(telemetryPrefix + " Set Speed Dashboard");
        } else {
            return idle();
        }
    }

    public Command setDutyCycle(double dutyCycle) {
        if (flyWheel != null && dutyCycle != 0) {
            return Commands.either(
                    Commands.sequence(
                            jammedCommand().withTimeout(.5),
                            Commands.waitSeconds(0.5),
                            Commands.runOnce(() -> {
                                isJammed = false;
                                timeJammed.reset();
                            })),
                    flyWheel.set(dutyCycle),
                    () -> isJammed)
                    .withName(telemetryPrefix + " Set Duty Cycle");
        }else if(dutyCycle == 0){
            return flyWheel.set(0);
        }
        return idle();
    }

    private Command jammedCommand() {
        return flyWheel.set(-0.5);
    }

    private void updateJammed() {
        AngularVelocity aVelocity = flyWheel.getMotor().getMechanismVelocity();
        Current current = flyWheel.getMotor().getStatorCurrent();

        Time jammed = Milliseconds.of(500);

        if (!timeJammed.isRunning())
            timeJammed.start();

        if (current.gte(Amps.of(30)) && aVelocity.lte(RotationsPerSecond.of(30))) {
            if (timeJammed.hasElapsed(jammed) && !isJammed) {
                isJammed = true;
            }
        } else {
            timeJammed.stop();
            timeJammed.reset();
        }
    }

    /*
     * public Command setDutyCycleGated(
     * double dutyCycle,
     * BooleanSupplier shooterAtRPM,
     * BooleanSupplier turretAtTarget,
     * BooleanSupplier shooterHoodAtTarget) {
     * if (flyWheel != null) {
     * 
     * BooleanSupplier readyToFeed = () -> turretAtTarget.getAsBoolean()
     * && shooterHoodAtTarget.getAsBoolean();
     * 
     * BooleanSupplier notReadyToFeed = () -> !turretAtTarget.getAsBoolean()
     * || !shooterHoodAtTarget.getAsBoolean();
     * 
     * return Commands.either(
     * flyWheel.set(dutyCycle),
     * flyWheel.set(0),
     * readyToFeed).repeatedly().withName("Conveyor Gated");
     * }
     * return idle();
     * }
     */

    public TalonFX getMotor() {
        if (motor != null) {
            return motor;
        }
        return null;
    }
}