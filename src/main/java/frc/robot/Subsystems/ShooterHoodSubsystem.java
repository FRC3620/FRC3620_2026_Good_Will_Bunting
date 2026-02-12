package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pound;
import static edu.wpi.first.units.Units.Rotations;

import org.usfirst.frc3620.CANDeviceType;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
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

    CANcoder shooterHoodEncoder;
    CANcoderConfiguration encoderConfiguration;

    Timer calibrationTimer;

    private final Angle HOOD_CALIBRATED_POS = Degrees.of(30); // place holders
    private final Angle MAXPOSITION = Degrees.of(60); // place holders

    private final Angle ZERO_ENCODER_OFFSET = Degrees.of(0); // place holders

    Double requestedCalibrationPos = null;

    public ShooterHoodSubsystem() {

        boolean makeDevices = RobotContainer.canDeviceFinder.isDevicePresent(CANDeviceType.TALON_PHOENIX6,
                motorId, telemetryPrefix) ||
                RobotContainer.shouldMakeAllCANDevices();

        if (makeDevices) {
            RobotContainer.canDeviceFinder.isDevicePresent(CANDeviceType.CANCODER_PHOENIX6, Constants.ENCODERID_HOOD);
            motor = new TalonFX(Constants.MOTORID_HOOD);
            shooterHoodEncoder = new CANcoder(Constants.ENCODERID_HOOD);

            encoderConfiguration = new CANcoderConfiguration();
            encoderConfiguration.MagnetSensor.withMagnetOffset(ZERO_ENCODER_OFFSET.in(Rotations));

            shooterHoodEncoder.getConfigurator().apply(encoderConfiguration);

            SmartMotorControllerConfig hoodConfig = new SmartMotorControllerConfig(this)
                    .withClosedLoopController(150, 0, 0, DegreesPerSecond.of(100), DegreesPerSecondPerSecond.of(100))
                    .withMotorInverted(true)
                    .withGearing(new MechanismGearing(GearBox.fromReductionStages(115.625)))
                    .withIdleMode(MotorMode.BRAKE)
                    .withTelemetry("ShooterHoodMotor", TelemetryVerbosity.HIGH)
                    .withStatorCurrentLimit(Amps.of(40))
                    .withFeedforward(new ArmFeedforward(0.5, 0.2, 0.5, 0))
                    .withMechanismCircumference(Inches.of(20.5).times(Math.PI))
                    .withControlMode(ControlMode.CLOSED_LOOP);
                    //.withExternalEncoder(shooterHoodEncoder)
                    //.withExternalEncoderInverted(false)
                    //.withExternalEncoderGearing(1)
                    //.withUseExternalFeedbackEncoder(true);
            // .withMOI(Feet.of(4), Pound.of(4));
            motorController = new TalonFXWrapper(motor, DCMotor.getKrakenX60(1), hoodConfig);

            pivot = new Pivot(new PivotConfig(motorController)
                    .withHardLimit(HOOD_CALIBRATED_POS, MAXPOSITION)
                    .withSoftLimits(HOOD_CALIBRATED_POS, MAXPOSITION)
                    .withStartingPosition(HOOD_CALIBRATED_POS)
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
