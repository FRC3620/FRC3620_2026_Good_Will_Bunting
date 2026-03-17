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

public class IntakeRollerSubsytem extends SubsystemBase {
    int motorId = Constants.MOTORID_INTAKEROLLERS;
    String telemetryPrefix = "IntakeRollers";

    private TalonFX motor = null;
    private SmartMotorController motorController;
    private FlyWheel flyWheel;

    public IntakeRollerSubsytem() {
        boolean makeDevices = RobotContainer.canDeviceFinder.isDevicePresent(
                org.usfirst.frc3620.CANDeviceType.TALON_PHOENIX6,
                motorId, telemetryPrefix) || RobotContainer.shouldMakeAllCANDevices();

        if (makeDevices) {
            motor = new TalonFX(motorId);
            RobotContainer.healthSubsystem.addMotorToWatch(motor, telemetryPrefix, HealthSubsystem.healthOptionsForYAMS);

            SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
                    .withControlMode(ControlMode.CLOSED_LOOP)
                    .withGearing(new MechanismGearing(GearBox.fromTeeth(18,36)))
                    .withClosedLoopController(1.0, 0, 0, RPM.of(3000), RotationsPerSecondPerSecond.of(500))
                    .withIdleMode(MotorMode.COAST)
                    .withTelemetry("motor", TelemetryVerbosity.HIGH)
                    .withStatorCurrentLimit(Amps.of(40))
                    .withSupplyCurrentLimit(Amps.of(40))
                    .withMotorInverted(true);

            motorController = new TalonFXWrapper(motor, DCMotor.getKrakenX60(1), motorConfig);

            // Create the FlyWheel
            flyWheel = new FlyWheel(new FlyWheelConfig(motorController)
                    .withDiameter(Inch.of(1.5))
                    .withMass(Pound.of(0.5))
                    .withUpperSoftLimit(RPM.of(3000))
                    .withTelemetry(telemetryPrefix, TelemetryVerbosity.HIGH));
        }
    }

    public Command rollersOn() {
        // Only use YAMS control, not manual rollers.set()
        Command rv;
        if (flyWheel != null) {
            rv = flyWheel.setSpeed(RPM.of(2500)); // need to test this
        } else {
            rv = idle();
        }
        return rv.withName(telemetryPrefix + " On");
    }

    public Command rollersOff() {
        Command rv;
        if (flyWheel != null) {
            rv = flyWheel.setSpeed(RPM.of(0));
        } else {
            rv = idle();
        }
        return rv.withName(telemetryPrefix + " Off");
    }

    public Command rollersBackwards() {
        Command rv;
        if (flyWheel != null) {
            rv = flyWheel.setSpeed(RPM.of(-2500));
        } else {
            rv = idle();
        }
        return rv.withName(telemetryPrefix + " Backwards");
    }

    @Override
    public void periodic() {
        if (flyWheel != null) {
            flyWheel.updateTelemetry();
            SmartDashboard.putNumber("frc3620/"+ telemetryPrefix + "/Intake Velocity", flyWheel.getSpeed().in(RPM));
            SmartDashboard.putNumber("frc3620/"+ telemetryPrefix + "/Intake DutyCycle", motorController.getDutyCycle());
        }
    }


    @Override
    public void simulationPeriodic() {
        if (flyWheel != null) {
            flyWheel.simIterate();
        }
    }
}
