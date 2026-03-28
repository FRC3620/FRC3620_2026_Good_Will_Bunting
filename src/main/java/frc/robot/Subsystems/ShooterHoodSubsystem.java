package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pound;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.usfirst.frc3620.CANDeviceType;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Power;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Helpers.ShotCalculator;
import frc.robot.Helpers.VelocityVector;
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

    Timer calibrationTimer;

    boolean isCalibrated = false;
    boolean activeCalibrating = false;
    Command calibrationCommand;

    private final Voltage CALIBRATION_VOLTAGE = Volts.of(-1.5);
    private final double VELOCITY_THRESHOLD = 2.0; // deg/sec
    private final double STALL_TIME_SECONDS = 0.25;

    private final Angle HOOD_CALIBRATED_POS = Degrees.of(30); // place holders
    private final Angle MAXPOSITION = Degrees.of(60); // place holders

    private final Angle ZERO_ENCODER_OFFSET = Rotations.of(0.281982); // place holders

    Double requestedCalibrationPos = null;

    private boolean atTarget = false;
    Angle targetAngle = Degrees.of(0.0);

    public ShooterHoodSubsystem() {

        boolean makeDevices = RobotContainer.canDeviceFinder.isDevicePresent(CANDeviceType.TALON_PHOENIX6,
                motorId, telemetryPrefix) ||
                RobotContainer.shouldMakeAllCANDevices();

        if (makeDevices) {
            motor = new TalonFX(Constants.MOTORID_HOOD);
            // shooterHoodEncoder = new CANcoder(Constants.ENCODERID_HOOD);
            RobotContainer.healthSubsystem.addMotorToWatch(motor, telemetryPrefix,
                    HealthSubsystem.healthOptionsForYAMS);

            SmartMotorControllerConfig hoodConfig = new SmartMotorControllerConfig(this)
                    .withClosedLoopController(150, 0, 0, DegreesPerSecond.of(200), DegreesPerSecondPerSecond.of(200))
                    .withMotorInverted(true)
                    .withGearing(new MechanismGearing(GearBox.fromReductionStages(115.625)))
                    .withIdleMode(MotorMode.BRAKE)
                    .withTelemetry("ShooterHoodMotor", TelemetryVerbosity.LOW)
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
        SmartDashboard.putNumber("frc3620/ShotCalculator/HoodAlpha", 1.0);
        SmartDashboard.putBoolean("SHOOTER HOOD END RAN", false);
        SmartDashboard.putNumber("frc3620/ShotCalculator/Ratio Over Min Velocity", 1.03);
        SmartDashboard.putData(this);

    }

    @Override
    public void periodic() {
        if (pivot != null) {

            if (!isCalibrated && !activeCalibrating && !RobotBase.isSimulation()) {
                calibrationCommand = calibrate();
                CommandScheduler.getInstance().schedule(calibrationCommand);
            }

            pivot.updateTelemetry();

            SmartDashboard.putBoolean("frc3620/ShooterHood/atTarget", atTarget().getAsBoolean());

            SmartDashboard.putNumber("frc3620/ShooterHood/Voltage", motorController.getVoltage().in(Volts));
            SmartDashboard.putNumber("frc3620/ShooterHood/Hood Velocity Deg p SEc",
                    motorController.getMechanismVelocity().in(DegreesPerSecond));
            SmartDashboard.putNumber("frc3620/ShooterHood/Hood Stator Current",
                    motorController.getStatorCurrent().in(Amps));
            SmartDashboard.putNumber("frc3620/ShooterHood/Hood Angle Degrees", getAngle().in(Degrees));
            SmartDashboard.putBoolean("frc3620/ShooterHood/isCalibrated", isCalibrated);
            SmartDashboard.putBoolean("frc3620/ShooterHood/isCalibrating", activeCalibrating);
            // SmartDashboard.putNumber("frc3620/ShooterHood/Hood Angle Degrees Encoder",
            // Degrees.convertFrom(shooterHoodEncoder.getPosition().getValueAsDouble(),
            // Rotations));
        }
    }

    private void createPivot(Angle startingAngle) {
        pivot = new Pivot(new PivotConfig(motorController)
                .withHardLimit(HOOD_CALIBRATED_POS, MAXPOSITION)
                .withSoftLimits(HOOD_CALIBRATED_POS, MAXPOSITION)
                .withStartingPosition(startingAngle)
                .withMOI(Inches.of(55.7), Pound.of(1))
                .withTelemetry(telemetryPrefix, TelemetryVerbosity.LOW));
    }

    @Override
    public void simulationPeriodic() {
        if (pivot != null) {
            pivot.simIterate();
        }
    }

    public Command createSetAngleCommand(Supplier<Angle> angle) {
        Command rv;
        if (pivot != null) {
            rv = Commands.waitUntil(() -> isCalibrated).andThen(pivot.setAngle(() -> {
                targetAngle = angle.get();
                return targetAngle;
            }));
        } else {
            rv = idle();
        }
        return rv.withName("Shooter Hood setAngle");

    }

    public Command createSetAngleCommandGated(Supplier<Angle> angle) {
        Command rv;
        if (pivot != null) {
            rv = Commands.waitUntil(() -> isCalibrated).andThen(Commands.none())
                    .andThen(createSetAngleCommand(angle).asProxy());
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
        return createSetAngleCommandGated(
                () -> Degrees.of(SmartDashboard.getNumber("frc3620/ShooterHood/Hood Angle Dashboard Control", 30)))
                .withName("Shooter Hood setAngle Dashboard");
    }

    public Command createAutoAngleToTargetCommand(Translation3d targetPosition, Supplier<Pose2d> robotPosition,
            Supplier<VelocityVector> robotVelocity, Supplier<AngularVelocity> shooterSpeed) {
        if (pivot == null)
            return idle();

        return createSetAngleCommandGated(
                () -> {
                    Angle raw = ShotCalculator.calculateHoodAngle(targetPosition, robotPosition, robotVelocity,
                            shooterSpeed);
                    return raw;
                }).withName("Shooter Hood Auto Angle To Target");
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
                        stallStartTime > STALL_TIME_SECONDS);
                return (Timer.getFPGATimestamp() - stallStartTime) > STALL_TIME_SECONDS;
            }

            public void end(boolean interrupted) {
                motorController.setDutyCycle(0);
                activeCalibrating = false;

                SmartDashboard.putBoolean("SHOOTER HOOD END RAN", true);

                if (!interrupted) {
                    motorController.setEncoderPosition(HOOD_CALIBRATED_POS);
                    motorController.startClosedLoopController();

                    motorController.setPosition(HOOD_CALIBRATED_POS);

                    isCalibrated = true;

                }

            }
        }
                .withName("Shooter Hood Calibration");
    }

    public BooleanSupplier atTarget() {

        Angle current = getAngle();
        if (current.isNear(targetAngle, Degrees.of(5))) {
            atTarget = true;
        } else {
            atTarget = false;
        }

        return () -> atTarget;
    }

    public TalonFX getMotor() {
        if (motor != null) {
            return motor;
        }
        return null;
    }
}
