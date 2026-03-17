package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Pound;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

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

public class IntakeRollerSubsytem extends SubsystemBase {
    int motorId = Constants.MOTORID_INTAKEROLLERS;
    String telemetryPrefix = "IntakeRollers";

    private TalonFX motor = null;

    public IntakeRollerSubsytem() {
        boolean makeDevices = RobotContainer.canDeviceFinder.isDevicePresent(
                org.usfirst.frc3620.CANDeviceType.TALON_PHOENIX6,
                motorId, telemetryPrefix) || RobotContainer.shouldMakeAllCANDevices();

        if (makeDevices) {
            motor = new TalonFX(motorId);
            RobotContainer.healthSubsystem.addMotorToWatch(motor, telemetryPrefix, HealthSubsystem.healthOptionsForYAMS);

            TalonFXConfiguration config = new TalonFXConfiguration()
            .withMotionMagic(new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(RPM.of(3000))
                .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(50)));

            config.Voltage.withPeakForwardVoltage(12 * 0.95);
            config.Voltage.withPeakReverseVoltage(-12 * 0.95);
            config.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);

            motor.getConfigurator().apply(config);
            motor.setNeutralMode(NeutralModeValue.Brake);
        }
    }

    public Command rollersOn() {
        // Only use YAMS control, not manual rollers.set()
        Command rv;
        if (motor != null) {
            rv = run(() -> motor.set(0.9)); // need to test this
        } else {
            rv = idle();
        }
        return rv.withName(telemetryPrefix + " On");
    }

    public Command rollersOff() {
        Command rv;
        if (motor != null) {
            rv = run(() -> motor.set(0));
        } else {
            rv = idle();
        }
        return rv.withName(telemetryPrefix + " Off");
    }

    public Command rollersBackwards() {
        Command rv;
        if (motor != null) {
            rv = run(() -> motor.set(-0.9));
        } else {
            rv = idle();
        }
        return rv.withName(telemetryPrefix + " Backwards");
    }

    @Override
    public void periodic() {
        if (motor != null) {

        }
    }


    @Override
    public void simulationPeriodic() {

    }
}
