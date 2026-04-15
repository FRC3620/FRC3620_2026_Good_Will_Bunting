package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Pound;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

public class ConveyerSubsystem extends SubsystemBase {
    private enum JamState {
        NORMAL,
        JAM_DETECTED,
        REVERSING,
        WAITING_AFTER_REVERSE
    }

    private JamState jamState = JamState.NORMAL;
    private double jamStartTime = 0;
    private double reverseStartTime = 0;
    private double waitStartTime = 0;
    private double targetSpeedBeforeJam = 0;
    private Command currentConveyorCommand = null;

    // Tunable parameters
    private static final double JAM_CURRENT_THRESHOLD = 30.0;
    private static final double JAM_VELOCITY_THRESHOLD_RPM = 100.0;
    private static final double JAM_DETECTION_TIME = 0.15;
    private static final double REVERSE_DURATION = 0.2;
    private static final double WAIT_AFTER_REVERSE = 0.1;
    private static final double REVERSE_DUTY_CYCLE = -0.5;

    private boolean antiJamEnabled = true;

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
        SmartDashboard.putBoolean("frc3620/" + telemetryPrefix + "/AntiJamEnabled", antiJamEnabled);
    }

    public Command idle() {
        if (flyWheel != null) {
            return flyWheel.set(0).withName(telemetryPrefix + " Idle");
        }
        return Commands.none().withName(telemetryPrefix + " Idle (No Motor)");
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

    @Override
    public void periodic() {
        if (flyWheel != null) {
            flyWheel.updateTelemetry();
            SmartDashboard.putNumber("frc3620/" + telemetryPrefix + "/actualSpeedRPM", flyWheel.getSpeed().in(RPM));

            // Add anti-jam telemetry
            if (motor != null) {
                SmartDashboard.putNumber("frc3620/" + telemetryPrefix + "/StatorCurrent",
                        motor.getStatorCurrent().getValueAsDouble());
                SmartDashboard.putNumber("frc3620/" + telemetryPrefix + "/JamStateOrdinal",
                        jamState.ordinal());
                SmartDashboard.putBoolean("frc3620/" + telemetryPrefix + "/AntiJamEnabled",
                        antiJamEnabled);
                SmartDashboard.putString("frc3620/" + telemetryPrefix + "/JamState",
                        jamState.toString());
            }
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
        if (flyWheel != null) {
            // Create a command that monitors for jams and adjusts accordingly
            return Commands.sequence(
                // Start the conveyor running
                Commands.runOnce(() -> {
                    if (currentConveyorCommand != null) {
                        currentConveyorCommand.cancel();
                    }
                    currentConveyorCommand = flyWheel.set(dutyCycle);
                    currentConveyorCommand.schedule();
                }),
                // Monitor for jams while running
                Commands.run(() -> {
                    if (antiJamEnabled && motor != null) {
                        updateAntiJam(dutyCycle);
                        
                        // Handle state transitions
                        if (jamState == JamState.REVERSING && !isCurrentlyReversing()) {
                            // Start reversing
                            if (currentConveyorCommand != null) {
                                currentConveyorCommand.cancel();
                            }
                            currentConveyorCommand = flyWheel.set(REVERSE_DUTY_CYCLE);
                            currentConveyorCommand.schedule();
                        } else if (jamState == JamState.WAITING_AFTER_REVERSE && !isCurrentlyWaiting()) {
                            // Stop during wait period
                            if (currentConveyorCommand != null) {
                                currentConveyorCommand.cancel();
                            }
                            currentConveyorCommand = flyWheel.set(0);
                            currentConveyorCommand.schedule();
                        } else if (jamState == JamState.NORMAL && isCurrentlyReversingOrWaiting()) {
                            // Resume normal operation
                            if (currentConveyorCommand != null) {
                                currentConveyorCommand.cancel();
                            }
                            currentConveyorCommand = flyWheel.set(dutyCycle);
                            currentConveyorCommand.schedule();
                        }
                    }
                }, this)
            ).finallyDo(() -> {
                if (currentConveyorCommand != null) {
                    currentConveyorCommand.cancel();
                    currentConveyorCommand = null;
                }
                resetJamState();
            }).withName("Conveyer with AntiJam");
        }
        return idle();
    }

    public Command setDutyCycleGated(
            double dutyCycle,
            BooleanSupplier shooterAtRPM,
            BooleanSupplier turretAtTarget,
            BooleanSupplier shooterHoodAtTarget) {
        if (flyWheel != null) {
            BooleanSupplier readyToFeed = () -> turretAtTarget.getAsBoolean()
                    && shooterHoodAtTarget.getAsBoolean();
            
            return Commands.sequence(
                Commands.run(() -> {
                    boolean shouldFeed = readyToFeed.getAsBoolean();
                    
                    if (shouldFeed && antiJamEnabled) {
                        // Run with anti-jam protection
                        if (currentConveyorCommand != null && !isCurrentlyReversingOrWaiting()) {
                            updateAntiJam(dutyCycle);
                            
                            if (jamState == JamState.REVERSING && !isCurrentlyReversing()) {
                                if (currentConveyorCommand != null) currentConveyorCommand.cancel();
                                currentConveyorCommand = flyWheel.set(REVERSE_DUTY_CYCLE);
                                currentConveyorCommand.schedule();
                            } else if (jamState == JamState.WAITING_AFTER_REVERSE && !isCurrentlyWaiting()) {
                                if (currentConveyorCommand != null) currentConveyorCommand.cancel();
                                currentConveyorCommand = flyWheel.set(0);
                                currentConveyorCommand.schedule();
                            } else if (jamState == JamState.NORMAL && isCurrentlyReversingOrWaiting()) {
                                if (currentConveyorCommand != null) currentConveyorCommand.cancel();
                                currentConveyorCommand = flyWheel.set(dutyCycle);
                                currentConveyorCommand.schedule();
                            }
                        } else if (shouldFeed && !isCurrentlyReversingOrWaiting()) {
                            // Normal operation
                            if (currentConveyorCommand == null || currentConveyorCommand.isScheduled() == false) {
                                currentConveyorCommand = flyWheel.set(dutyCycle);
                                currentConveyorCommand.schedule();
                            }
                        }
                    } else if (!shouldFeed) {
                        // Not ready to feed - stop
                        if (currentConveyorCommand != null) {
                            currentConveyorCommand.cancel();
                            currentConveyorCommand = null;
                        }
                        resetJamState();
                        flyWheel.set(0).schedule();
                    }
                }, this)
            ).finallyDo(() -> {
                if (currentConveyorCommand != null) {
                    currentConveyorCommand.cancel();
                    currentConveyorCommand = null;
                }
                resetJamState();
            }).withName("Conveyor Gated with AntiJam");
        }
        return idle();
    }

    public TalonFX getMotor() {
        if (motor != null) {
            return motor;
        }
        return null;
    }

    // Core anti-jam logic
    private void updateAntiJam(double commandedDutyCycle) {
        if (!antiJamEnabled || motor == null) {
            return;
        }

        double currentAmps = motor.getStatorCurrent().getValueAsDouble();
        double currentRPM = getCurrentRPM();
        boolean isJammed = isMechanismJammed(currentAmps, currentRPM, commandedDutyCycle);

        switch (jamState) {
            case NORMAL:
                if (isJammed && Math.abs(commandedDutyCycle) > 0.1) {
                    if (jamStartTime == 0) {
                        jamStartTime = Timer.getFPGATimestamp();
                    } else if (Timer.getFPGATimestamp() - jamStartTime >= JAM_DETECTION_TIME) {
                        jamState = JamState.JAM_DETECTED;
                        targetSpeedBeforeJam = commandedDutyCycle;
                    }
                } else {
                    jamStartTime = 0;
                }
                break;

            case JAM_DETECTED:
                jamState = JamState.REVERSING;
                reverseStartTime = Timer.getFPGATimestamp();
                break;

            case REVERSING:
                if (Timer.getFPGATimestamp() - reverseStartTime >= REVERSE_DURATION) {
                    jamState = JamState.WAITING_AFTER_REVERSE;
                    waitStartTime = Timer.getFPGATimestamp();
                }
                break;

            case WAITING_AFTER_REVERSE:
                if (Timer.getFPGATimestamp() - waitStartTime >= WAIT_AFTER_REVERSE) {
                    jamState = JamState.NORMAL;
                    jamStartTime = 0;
                }
                break;
        }
    }

    private boolean isMechanismJammed(double currentAmps, double currentRPM, double commandedDutyCycle) {
        if (Math.abs(commandedDutyCycle) < 0.1) {
            return false;
        }

        boolean highCurrent = currentAmps > JAM_CURRENT_THRESHOLD;
        boolean lowVelocity = Math.abs(currentRPM) < JAM_VELOCITY_THRESHOLD_RPM;
        boolean directionMismatch = (commandedDutyCycle > 0 && currentRPM < -JAM_VELOCITY_THRESHOLD_RPM) ||
                (commandedDutyCycle < 0 && currentRPM > JAM_VELOCITY_THRESHOLD_RPM);

        return (highCurrent && lowVelocity) || directionMismatch;
    }

    private double getCurrentRPM() {
        if (motor != null && flyWheel != null) {
            return flyWheel.getSpeed().in(RPM);
        }
        return 0;
    }

    private void resetJamState() {
        jamState = JamState.NORMAL;
        jamStartTime = 0;
        reverseStartTime = 0;
        waitStartTime = 0;
        targetSpeedBeforeJam = 0;
    }

    private boolean isCurrentlyReversing() {
        return jamState == JamState.REVERSING;
    }

    private boolean isCurrentlyWaiting() {
        return jamState == JamState.WAITING_AFTER_REVERSE;
    }

    private boolean isCurrentlyReversingOrWaiting() {
        return isCurrentlyReversing() || isCurrentlyWaiting();
    }

    public Command manualClearJam() {
        return Commands.runOnce(() -> {
            resetJamState();
            if (currentConveyorCommand != null) {
                currentConveyorCommand.cancel();
                currentConveyorCommand = null;
            }
        }).withName("Manual Clear Jam");
    }

    public Command manualReverse(double duration, double dutyCycle) {
        return Commands.sequence(
            Commands.runOnce(() -> {
                if (currentConveyorCommand != null) {
                    currentConveyorCommand.cancel();
                }
            }),
            flyWheel.set(dutyCycle).withTimeout(duration),
            Commands.runOnce(() -> resetJamState())
        ).withName("Manual Reverse");
    }

    public Command toggleAntiJam() {
        return Commands.runOnce(() -> {
            antiJamEnabled = !antiJamEnabled;
            if (!antiJamEnabled) {
                resetJamState();
            }
            SmartDashboard.putBoolean("frc3620/" + telemetryPrefix + "/AntiJamEnabled", antiJamEnabled);
        }).withName("Toggle Anti-Jam");
    }
}