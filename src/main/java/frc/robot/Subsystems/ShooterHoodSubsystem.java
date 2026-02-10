package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pound;

import org.usfirst.frc3620.CANDeviceType;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Arm;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXSWrapper;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class ShooterHoodSubsystem extends SubsystemBase {

    TalonFX motor = null;
    private Pivot pivot = null;
    private SmartMotorController motorController = null;

    public ShooterHoodSubsystem() {

        boolean makeDevices = RobotContainer.canDeviceFinder.isDevicePresent(CANDeviceType.TALON_PHOENIX6,
                Constants.MOTORID_HOOD) ||
                RobotContainer.shouldMakeAllCANDevices();

        if (makeDevices) {
            motor = new TalonFX(Constants.MOTORID_HOOD);
            SmartMotorControllerConfig hoodConfig = new SmartMotorControllerConfig(this)
                    .withClosedLoopController(4, 0, 0, DegreesPerSecond.of(5), DegreesPerSecondPerSecond.of(2.5))
                    .withSoftLimit(Degrees.of(25), Degrees.of(65))
                    .withGearing(new MechanismGearing(GearBox.fromReductionStages(115.625, 1)))
                    .withIdleMode(MotorMode.BRAKE)
                    .withTelemetry("ShooterHoodMotor", TelemetryVerbosity.HIGH)
                    .withStatorCurrentLimit(Amps.of(5))
                    .withFeedforward(new ArmFeedforward(0, 0, 0, 0))
                    .withControlMode(ControlMode.CLOSED_LOOP);
            // .withMOI(Feet.of(4), Pound.of(4));
            motorController = new TalonFXWrapper(motor, DCMotor.getKrakenX60(1), hoodConfig);
            pivot = new Pivot(new PivotConfig(motorController)
                    .withHardLimit(Degrees.of(10), Degrees.of(80))
                    .withStartingPosition(Degrees.of(10))
                    .withMOI(Inches.of(55.7), Pound.of(1))
                    .withTelemetry("Shooter Hood", TelemetryVerbosity.HIGH)
            );}

    }

    @Override
    public void periodic() {
        if (pivot != null) {
            pivot.updateTelemetry();
        }
    }

    @Override
    public void simulationPeriodic() {
        // setAngle(intake /\)
        if (pivot != null) {
            pivot.simIterate();
        }
    }

    public Command setAngle(Angle angle){
        if (pivot != null)
            return pivot.setAngle(angle).withName("Shooter Hood setAngle");
        else
            return this.run(() -> { 
                // RobotContainer.logger.error("Shooter Hood not initialized");
            }).withName("Shooter Hood setAngle");
    }
}
