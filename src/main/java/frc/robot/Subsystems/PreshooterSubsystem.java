package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Pound;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.TalonFX;

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

public class PreshooterSubsystem extends SubsystemBase {
    int motorId = Constants.MOTORID_PRESHOOTER;
    String telemetryPrefix = "Preshooter";

    private TalonFX motor = null;
    private SmartMotorController motorController;
    private FlyWheel flyWheel;

    private AngularVelocity setpoint = RPM.of(0);

    public PreshooterSubsystem() {
        boolean makeDevices = RobotContainer.canDeviceFinder.isDevicePresent(
                org.usfirst.frc3620.CANDeviceType.TALON_PHOENIX6,
                motorId, telemetryPrefix) || RobotContainer.shouldMakeAllCANDevices();
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
                    .withGearing(new MechanismGearing(GearBox.fromReductionStages(1, 1))) // Direct drive
                    .withIdleMode(MotorMode.BRAKE)
                    .withTelemetry("motor", TelemetryVerbosity.HIGH)
                    .withStatorCurrentLimit(Amps.of(40))
                    .withSupplyCurrentLimit(Amps.of(40))
                    .withControlMode(ControlMode.CLOSED_LOOP);

            motorController = new TalonFXWrapper(motor, DCMotor.getKrakenX60(2), motorConfig);

            // Create the FlyWheel
            flyWheel = new FlyWheel(new FlyWheelConfig(motorController)
                    .withDiameter(Inch.of(4))
                    .withMass(Pound.of(0.5))
                    .withUpperSoftLimit(RPM.of(2000))
                    .withTelemetry(telemetryPrefix, TelemetryVerbosity.HIGH));

            setDefaultCommand(flyWheel.setSpeed(() -> setpoint));
        }
        SmartDashboard.putNumber("frc3620/" + telemetryPrefix + "/RPM Dashboard Control", 0);
    }

    /*
     * @parma speed speed to set
     * 
     * @return {@link edu.wpi.frist.wpilibj.command.Runcommand}
     */
    public Command setVelocityCommand(Supplier<AngularVelocity> speed) {
        Command rv;
        if (flyWheel == null) {
            rv = idle();
        } else {
            rv = run(() -> {
                setpoint = speed.get();
            });
        }
        return rv.withName(telemetryPrefix + " SetVelocity");
    }

    public Command setVelocityDashboardCommand() {
        Command rv;
        if (flyWheel == null) {
            rv = idle();
        } else {
            rv = run(() -> {
                setpoint = RPM.of(SmartDashboard.getNumber("frc3620/" + telemetryPrefix + "/RPM Dashboard Control", 0));
            });
        }
        return rv.withName(telemetryPrefix + " SetVelocityDashboard");
    }

     /*
     * @parma dutyCycle duty cycle to set
     * 
     * @return {@link edu.wpi.frist.wpilibj.command.Runcommand}

    // ** */
    public Command setDutyCycleCommand(double dutyCycle) {
        Command rv;
        if (flyWheel == null) {
            rv = idle();
        } else {
            rv = flyWheel.set(dutyCycle);
        }
        return rv.withName(telemetryPrefix + " SetDutyCycle");
    }

    @Override
    public void periodic() {
        if (flyWheel != null) {
            flyWheel.updateTelemetry();
            SmartDashboard.putNumber("frc3620/" + telemetryPrefix + "/setVelocity", setpoint.in(RPM));
            SmartDashboard.putNumber("frc3620/" + telemetryPrefix + "/actualVelocity", flyWheel.getSpeed().in(RPM));
        }
    }

    public void simulationPeriodic() {
        if (flyWheel != null) {
            flyWheel.simIterate();
        }
    }

}
