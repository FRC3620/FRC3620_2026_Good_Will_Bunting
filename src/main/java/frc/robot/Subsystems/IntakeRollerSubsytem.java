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
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
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
    int motorId1 = Constants.MOTORID_INTAKEROLLERS1;
    int motorId2 = Constants.MOTORID_INTAKEROLLERS2;
    String telemetryPrefix = "IntakeRollers";

    private TalonFX motor1 = null;
    private TalonFX motor2 = null;

    public IntakeRollerSubsytem() {
        boolean makeDevices = (RobotContainer.canDeviceFinder.isDevicePresent(
                org.usfirst.frc3620.CANDeviceType.TALON_PHOENIX6,
                motorId1, telemetryPrefix) &&
                RobotContainer.canDeviceFinder.isDevicePresent(
                    org.usfirst.frc3620.CANDeviceType.TALON_PHOENIX6,
                motorId2, telemetryPrefix + " Follower")) || RobotContainer.shouldMakeAllCANDevices();

        if (makeDevices) {
            motor1 = new TalonFX(motorId1);
            motor2 = new TalonFX(motorId2);
            RobotContainer.healthSubsystem.addMotorToWatch(motor1, telemetryPrefix, HealthSubsystem.healthOptionsForYAMS);
            RobotContainer.healthSubsystem.addMotorToWatch(motor2, telemetryPrefix, HealthSubsystem.healthOptionsForYAMS);

            TalonFXConfiguration config = new TalonFXConfiguration()
            .withMotionMagic(new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(RPM.of(3000))
                .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(50)));

            config.Voltage.withPeakForwardVoltage(12 * 0.8);
            config.Voltage.withPeakReverseVoltage(-12 * 0.8);
            config.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);

            motor1.setNeutralMode(NeutralModeValue.Brake);
            motor1.getConfigurator().apply(config);

            motor2.setControl(new Follower(motorId1, MotorAlignmentValue.Opposed));
        }
    }

    public Command rollersOn() {
        // Only use YAMS control, not manual rollers.set()
        Command rv;
        if (motor1 != null) {
            rv = run(() -> motor1.set(0.80)); // need to test this
        } else {
            rv = idle();
        }
        return rv.withName(telemetryPrefix + " On");
    }

    public Command rollersOff() {
        Command rv;
        if (motor1 != null) {
            rv = run(() -> motor1.set(0));
        } else {
            rv = idle();
        }
        return rv.withName(telemetryPrefix + " Off");
    }

    public Command rollersBackwards() {
        Command rv;
        if (motor1 != null) {
            rv = run(() -> motor1.set(-0.90));
        } else {
            rv = idle();
        }
        return rv.withName(telemetryPrefix + " Backwards");
    }

    @Override
    public void periodic() {
        if (motor1 != null) {
            SmartDashboard.putNumber("frc3620/IntakeRollers/Rotor Velocity RPS (about 2x drum speed)", motor1.getVelocity().getValueAsDouble());
            SmartDashboard.putNumber("frc3620/IntakeRollers/Supply Current Amps", motor1.getSupplyCurrent().getValueAsDouble());
        }
    }


    @Override
    public void simulationPeriodic() {

    }
}
