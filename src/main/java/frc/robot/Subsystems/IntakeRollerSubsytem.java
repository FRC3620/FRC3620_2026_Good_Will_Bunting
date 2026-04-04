package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Pound;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.Pair;
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

public class IntakeRollerSubsytem extends SubsystemBase {
    int motorId1 = Constants.MOTORID_INTAKEROLLERS2;
    int motorId2 = Constants.MOTORID_INTAKEROLLERS1;
    String telemetryPrefix = "IntakeRollers";

    private TalonFX motor1 = null;
    private TalonFX motor2 = null;

    private SmartMotorController motorController;

    private FlyWheel flyWheel;

    public IntakeRollerSubsytem() {
        boolean makeDevices = (RobotContainer.canDeviceFinder.isDevicePresent(
                org.usfirst.frc3620.CANDeviceType.TALON_PHOENIX6,
                motorId1, telemetryPrefix) &&
                RobotContainer.canDeviceFinder.isDevicePresent(
                        org.usfirst.frc3620.CANDeviceType.TALON_PHOENIX6,
                        motorId2, telemetryPrefix + " Follower"))
                || RobotContainer.shouldMakeAllCANDevices();

        if (makeDevices) {
            motor1 = new TalonFX(motorId1);
            motor2 = new TalonFX(motorId2);
            RobotContainer.healthSubsystem.addMotorToWatch(motor1, telemetryPrefix,
                    HealthSubsystem.healthOptionsForYAMS);
            RobotContainer.healthSubsystem.addMotorToWatch(motor2, telemetryPrefix,
                    HealthSubsystem.healthOptionsForYAMS);

            SmartMotorControllerConfig config = new SmartMotorControllerConfig(this)
                    .withClosedLoopController(10, 0, 0, RPM.of(6000), RotationsPerSecondPerSecond.of(100))
                    .withSimClosedLoopController(10, 0, 0, RPM.of(6000), RotationsPerSecondPerSecond.of(100))
                    .withMotorInverted(false)
                    .withFollowers(Pair.of(motor2, true)) // motor2 follows motor1, inverted
                    .withGearing(new MechanismGearing(GearBox.fromTeeth(18,30))) // Direct drive
                    .withIdleMode(MotorMode.BRAKE)
                    .withTelemetry(telemetryPrefix + " Motor", TelemetryVerbosity.LOW)
                    .withStatorCurrentLimit(Amps.of(40))
                    .withSupplyCurrentLimit(Amps.of(40))
                    .withControlMode(ControlMode.CLOSED_LOOP);

            motorController = new TalonFXWrapper(motor1, DCMotor.getKrakenX60(1), config);
            FlyWheelConfig rollerConfig = new FlyWheelConfig(motorController)
                    .withDiameter(Inch.of(4))
                    .withMass(Pound.of(0.5))
                    .withUpperSoftLimit(RPM.of(7000))
                    .withTelemetry(telemetryPrefix + " Roller", TelemetryVerbosity.LOW);

            // Create the FlyWheel
            flyWheel = new FlyWheel(rollerConfig);

            setDefaultCommand(idle());

            SmartDashboard.putNumber("frc3620/IntakeRollers/Flywheel RPM Dashboard Control", 0);
        }
    }

    /* public Command rollersOn() {
        // Only use YAMS control, not manual rollers.set()
        Command rv;
        if (flyWheel != null) {
            rv = flyWheel.setSpeed(RPM.of(1000)); // need to test this
        } else {
            rv = idle();
        }
        return rv.withName(telemetryPrefix + " On");
    } */

    public Command rollersOff() {
        Command rv;
        if (flyWheel != null) {
            rv = flyWheel.set(0);
        } else {
            rv = idle();
        }
        return rv.withName(telemetryPrefix + " Off");
    }

   /*  public Command rollersBackwards() {
        Command rv;
        if (flyWheel != null) {
            rv = flyWheel.setSpeed(RPM.of(-1000));
        } else {
            rv = idle();
        }
        return rv.withName(telemetryPrefix + " Backwards");
    } */

    public Command createSetVelocityCommand(Supplier<AngularVelocity> velocity) {
        if (flyWheel != null) {
            return flyWheel.setSpeed(velocity);
        }
        return idle();
    }

    public Command setVelocityDashboardCommand() {
    if (flyWheel == null)
      return idle();

    return createSetVelocityCommand(
        () -> RPM.of(SmartDashboard.getNumber("frc3620/IntakeRollers/Flywheel RPM Dashboard Control", 0)));

  }

    @Override
    public void periodic() {
        if (motor1 != null) {
            SmartDashboard.putNumber("frc3620/IntakeRollers/Rotor Velocity RPS (about 2x drum speed)",
                    motor1.getVelocity().getValueAsDouble());
            SmartDashboard.putNumber("frc3620/IntakeRollers/Supply Current Amps",
                    motor1.getSupplyCurrent().getValueAsDouble());
            SmartDashboard.putNumber("frc3620/IntakeRollers/Supply Current Amps",
                    motor2.getSupplyCurrent().getValueAsDouble());
        }
    }

    @Override
    public void simulationPeriodic() {

    }

    public TalonFX getMotor1() {
        if (motor1 != null) {
            return motor1;
        }
        return null;
    }

    public TalonFX getMotor2() {
        if (motor2 != null) {
            return motor2;
        }
        return null;
    }
}
