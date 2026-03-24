package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Pound;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.system.plant.DCMotor;
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

public class IntakeAgitatorSubsytem extends SubsystemBase {
    int motorId = Constants.MOTORID_INTAKEAGITATOR;
    String telemetryPrefix = "IntakeAGITATOR";

    private TalonFX motor = null;
    private SmartMotorController motorController;
    private FlyWheel flyWheel;

    public IntakeAgitatorSubsytem() {
        boolean makeDevices = RobotContainer.canDeviceFinder.isDevicePresent(
                org.usfirst.frc3620.CANDeviceType.TALON_PHOENIX6,
                motorId, telemetryPrefix) || RobotContainer.shouldMakeAllCANDevices();

        if (makeDevices) {
            motor = new TalonFX(motorId);
            RobotContainer.healthSubsystem.addMotorToWatch(motor, telemetryPrefix, HealthSubsystem.healthOptionsForYAMS);

            SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
                    .withGearing(new MechanismGearing(GearBox.fromTeeth(18,30)))// need to verify gearing here
                    .withIdleMode(MotorMode.BRAKE)
                    .withTelemetry("motor", TelemetryVerbosity.LOW)
                    .withStatorCurrentLimit(Amps.of(40))
                    .withSupplyCurrentLimit(Amps.of(40))
                    .withControlMode(ControlMode.OPEN_LOOP)
                    .withMotorInverted(false);

            motorController = new TalonFXWrapper(motor, DCMotor.getKrakenX60(1), motorConfig);

            // Create the FlyWheel
            flyWheel = new FlyWheel(new FlyWheelConfig(motorController)
                    .withDiameter(Inch.of(1.5))
                    .withMass(Pound.of(0.5))
                    .withUpperSoftLimit(RPM.of(2000))
                    .withTelemetry(telemetryPrefix, TelemetryVerbosity.LOW));
        }
    }

    public Command agitatorOn() {
        // Only use YAMS control, not manual rollers.set()
        Command rv;
        if (flyWheel != null) {
            rv = flyWheel.set(.10); // need to test this
        } else {
            rv = idle();
        }
        return rv.withName(telemetryPrefix + " On");
    }

    public Command agitatorOff() {
        Command rv;
        if (flyWheel != null) {
            rv = flyWheel.set(0);
        } else {
            rv = idle();
        }
        return rv.withName(telemetryPrefix + " Off");
    }

    public Command agitatorBackwards() {
        Command rv;
        if (flyWheel != null) {
            rv = flyWheel.set(-.10);
        } else {
            rv = idle();
        }
        return rv.withName(telemetryPrefix + " Backwards");
    }

    @Override
    public void periodic() {
        if (flyWheel != null) {
            flyWheel.updateTelemetry();
            SmartDashboard.putNumber(telemetryPrefix + "agitator Velocity", flyWheel.getSpeed().in(RPM));
        }
    }


    @Override
    public void simulationPeriodic() {
        if (flyWheel != null) {
            flyWheel.simIterate();
        }
    }

    public TalonFX getMotor() {
        if (motor != null) {
            return motor;
        }
        return null;
    }
}
