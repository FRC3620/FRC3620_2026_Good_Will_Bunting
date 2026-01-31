package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Pound;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.hal.CANAPITypes.CANDeviceType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DutyCycle;
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
import org.usfirst.frc3620.CANDeviceFinder;

public class PreshooterSubsystem extends SubsystemBase {
    private TalonFX motor = null;
    private SmartMotorController motorController;
    private FlyWheel flyWheel;

    public PreshooterSubsystem() {
        boolean makeDevices = RobotContainer.canDeviceFinder.isDevicePresent(
                org.usfirst.frc3620.CANDeviceType.TALON_PHOENIX6,
                Constants.MOTORID_PRESHOOTER, "Preshooter") || RobotContainer.shouldMakeAllCANDevices();
        if (makeDevices) {
            motor = new TalonFX(Constants.MOTORID_PRESHOOTER);
            SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
                    .withClosedLoopController(
                            0.0, // kP - tune this
                            0.0, // kI
                            0.0, // kD
                            RotationsPerSecond.of(0),
                            RotationsPerSecondPerSecond.of(0))
                    .withGearing(new MechanismGearing(GearBox.fromReductionStages(1, 1))) // Direct drive
                    .withIdleMode(MotorMode.BRAKE)
                    .withTelemetry("PreshooterMotor", TelemetryVerbosity.HIGH)
                    .withStatorCurrentLimit(Amps.of(40))
                    .withSupplyCurrentLimit(Amps.of(40))
                    .withControlMode(ControlMode.CLOSED_LOOP);

            motorController = new TalonFXWrapper(motor, DCMotor.getKrakenX60(2), motorConfig);
            FlyWheelConfig rollerConfig = new FlyWheelConfig(motorController)
                    .withDiameter(Inch.of(0))
                    .withMass(Pound.of(0))
                    .withUpperSoftLimit(RPM.of(0))
                    .withTelemetry("Preshooter", TelemetryVerbosity.HIGH);

            // Create the FlyWheel
            flyWheel = new FlyWheel(rollerConfig);

        }
    }

    /*
     * @parma speed speed to set
     * 
     * @return {@link edu.wpi.frist.wpilibj.command.Runcommand}
     */
    public Command setVelocityCommand(AngularVelocity speed) {
        if (flyWheel == null) {
            return donothingcommand();
        } else {
            return flyWheel.setSpeed(speed);
        }
    }

    // ** */
    public Command setDutyCycleCommand(double dutyCycle) {
        if (flyWheel == null) {
            return donothingcommand();
        } else {
            return flyWheel.set(dutyCycle);
        }
    }

    private Command donothingcommand() {
        // idk
        throw new UnsupportedOperationException("Unimplemented method 'donothingcommand'");
    }

    @Override
    public void periodic() {
        if (flyWheel != null)
            flyWheel.updateTelemetry();
    }

    public void simulationPeriodic() {
        if (flyWheel != null) {
            flyWheel.simIterate();
        }

    }

}
