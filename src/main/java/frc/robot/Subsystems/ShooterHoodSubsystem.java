package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pound;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import org.usfirst.frc3620.CANDeviceType;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Power;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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

    boolean isCalibrated = false;
    boolean activeCalibrating = false;
    Command calibrationCommand;

    private final Voltage CALIBRATION_VOLTAGE = Volts.of(-0.5);
    private final double VELOCITY_THRESHOLD = 2.0; // deg/sec
    private final double STALL_TIME_SECONDS = 0.25;

    private final Angle HOOD_CALIBRATED_POS = Degrees.of(30); // place holders
    private final Angle MAXPOSITION = Degrees.of(60); // place holders

    private final Angle ZERO_ENCODER_OFFSET = Rotations.of(0.281982); // place holders

    private Angle setpoint = Degrees.of(30);

    Double requestedCalibrationPos = null;

    public ShooterHoodSubsystem() {

        boolean makeDevices = RobotContainer.canDeviceFinder.isDevicePresent(CANDeviceType.TALON_PHOENIX6,
                motorId, telemetryPrefix) ||
                RobotContainer.shouldMakeAllCANDevices();

        if (makeDevices) {
            RobotContainer.canDeviceFinder.isDevicePresent(CANDeviceType.CANCODER_PHOENIX6, Constants.ENCODERID_HOOD,
                    telemetryPrefix);
            motor = new TalonFX(Constants.MOTORID_HOOD);
            shooterHoodEncoder = new CANcoder(Constants.ENCODERID_HOOD);

            SmartMotorControllerConfig hoodConfig = new SmartMotorControllerConfig(this)
                    .withClosedLoopController(150, 0, 0, DegreesPerSecond.of(100), DegreesPerSecondPerSecond.of(100))
                    .withMotorInverted(true)
                    .withGearing(new MechanismGearing(GearBox.fromReductionStages(115.625)))
                    .withIdleMode(MotorMode.BRAKE)
                    .withTelemetry("ShooterHoodMotor", TelemetryVerbosity.HIGH)
                    .withStatorCurrentLimit(Amps.of(40))
                    // .withFeedforward(new ArmFeedforward(0.5, 0.2, 0.5, 0))
                    // .withMechanismCircumference(Inches.of(20.5).times(Math.PI))
                    .withControlMode(ControlMode.CLOSED_LOOP)
            /*
             * .withExternalEncoder(shooterHoodEncoder)
             * .withExternalEncoderZeroOffset(ZERO_ENCODER_OFFSET)
             * .withExternalEncoderInverted(false)
             * .withExternalEncoderGearing(360/29)
             * .withUseExternalFeedbackEncoder(true);
             * //.withMOI(Feet.of(4), Pound.of(4))
             */;
            motorController = new TalonFXWrapper(motor, DCMotor.getKrakenX60(1), hoodConfig);

            createPivot(MAXPOSITION);

            setDefaultCommand(idle().withName("Hood default command"));
        }
        SmartDashboard.putNumber("frc3620/ShooterHood/Hood Angle Dashboard Control", 30);
        SmartDashboard.putBoolean("SHOOTER HOOD END RAN", false);

        SmartDashboard.putData(this);

    }

    @Override
    public void periodic() {
        if (pivot != null) {

            if (!isCalibrated && !activeCalibrating) {
                calibrationCommand = calibrate();
                CommandScheduler.getInstance().schedule(calibrationCommand);
            }

            pivot.updateTelemetry();
            SmartDashboard.putNumber("frc3620/ShooterHood/Voltage", motorController.getVoltage().in(Volts));
            SmartDashboard.putNumber("frc3620/ShooterHood/Hood Velocity Deg p SEc",
                    motorController.getMechanismVelocity().in(DegreesPerSecond));
            SmartDashboard.putNumber("frc3620/ShooterHood/Hood Stator Current",
                    motorController.getStatorCurrent().in(Amps));
            SmartDashboard.putNumber("frc3620/ShooterHood/Hood Angle Degrees", getAngle().in(Degrees));
            SmartDashboard.putNumber("frc3620/ShooterHood/Hood Angle Degrees Setpoint", setpoint.in(Degrees));
            SmartDashboard.putBoolean("frc3620/ShooterHood/isCalibrated", isCalibrated);
            SmartDashboard.putBoolean("frc3620/ShooterHood/isCalibrating", activeCalibrating);
            SmartDashboard.putNumber("frc3620/ShooterHood/Hood Angle Degrees Encoder",
                    Degrees.convertFrom(shooterHoodEncoder.getPosition().getValueAsDouble(), Rotations));
        }
    }

    private void createPivot(Angle startingAngle) {
        pivot = new Pivot(new PivotConfig(motorController)
                .withHardLimit(HOOD_CALIBRATED_POS, MAXPOSITION)
                .withSoftLimits(HOOD_CALIBRATED_POS, MAXPOSITION)
                .withStartingPosition(startingAngle)
                .withMOI(Inches.of(55.7), Pound.of(1))
                .withTelemetry(telemetryPrefix, TelemetryVerbosity.HIGH));
    }

    @Override
    public void simulationPeriodic() {
        if (pivot != null) {
            pivot.simIterate();
        }
    }

    public Command setAngle(Supplier<Angle> angle) {
        Command rv;
        if (pivot != null) {
            rv = pivot.setAngle(angle);
        } else {
            rv = idle();
        }
        return rv.withName("Shooter Hood setAngle");

    }

    public Angle getAngle() {
        if (pivot != null) {
            return pivot.getAngle();
        } else {
            return Degrees.of(999999999);
        }

    }

    public Command setAngleDashboardCommand() {
        return setAngle(() -> Degrees.of(SmartDashboard.getNumber("frc3620/ShooterHood/Hood Angle Dashboard Control", 30))).withName("Shooter Hood setAngle Dashboard");
    }

    public Command calibrate() {
        return new Command() {

            {
                addRequirements(ShooterHoodSubsystem.this);
            }

            private double stallStartTime = -1;

            public void initialize() {
                // setDefaultCommand(idle());
                activeCalibrating = true;
                isCalibrated = false;
                stallStartTime = -1;
                motorController.stopClosedLoopController();
            }

            public void execute() {
                motorController.setVoltage(CALIBRATION_VOLTAGE);
                // motorController.setDutyCycle(-0.2);

                double velocity = motorController.getMechanismVelocity().in(DegreesPerSecond);
                double current = motorController.getStatorCurrent().in(Amps);

                if (Math.abs(velocity) < VELOCITY_THRESHOLD && current > 10) {
                    if (stallStartTime < 0) {
                        stallStartTime = Timer.getFPGATimestamp();
                    }
                } else {
                    stallStartTime = -1;
                }

                SmartDashboard.putNumber("frc3620/ShooterHood/Timer", Timer.getFPGATimestamp() - stallStartTime);
            }

            public boolean isFinished() {
                if (stallStartTime < 0)
                    return false;

                SmartDashboard.putBoolean("frc3620/ShooterHood/ShouldFinish",
                        (Timer.getFPGATimestamp() - stallStartTime) > STALL_TIME_SECONDS);
                return (Timer.getFPGATimestamp() - stallStartTime) > STALL_TIME_SECONDS;
            }

            public void end(boolean interrupted) {
                motorController.setDutyCycle(0);
                
                SmartDashboard.putBoolean("SHOOTER HOOD END RAN", true);
                motorController.setEncoderPosition(HOOD_CALIBRATED_POS);
                motorController.startClosedLoopController();

                motorController.setPosition(HOOD_CALIBRATED_POS);

                isCalibrated = true;
                activeCalibrating = false;

            }
        }
                .withName("Shooter Hood Calibration");
    }

}
