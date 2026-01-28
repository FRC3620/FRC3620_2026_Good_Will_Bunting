package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Pound;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
    
    private final TalonFX rollers;
    private final SmartMotorController rollerMotor;
    private final FlyWheel roller;
    
    public IntakeRollerSubsytem() {
        rollers = new TalonFX(57);
        
        // Configure the motor controller FIRST
        SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
            .withClosedLoopController(
                0.1,  // kP - tune this
                0.0,  // kI
                0.0,  // kD
                RotationsPerSecond.of(100),
                RotationsPerSecondPerSecond.of(200)
            )
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(1, 1))) // Direct drive
            .withIdleMode(MotorMode.BRAKE)
            .withTelemetry("RollerMotor", TelemetryVerbosity.HIGH)
            .withStatorCurrentLimit(Amps.of(40))
            .withSupplyCurrentLimit(Amps.of(40))
            .withControlMode(ControlMode.CLOSED_LOOP);
        
        // Create the motor controller wrapper
        rollerMotor = new TalonFXWrapper(rollers, DCMotor.getKrakenX60(1), motorConfig);
        
        // NOW create the FlyWheel config WITH the motor controller
        FlyWheelConfig rollerConfig = new FlyWheelConfig(rollerMotor)
            .withDiameter(Inch.of(4))
            .withMass(Pound.of(0.5))
            .withUpperSoftLimit(RPM.of(2000))
            .withTelemetry("Intake Roller", TelemetryVerbosity.HIGH);
        
        // Create the FlyWheel
        roller = new FlyWheel(rollerConfig);
    }
    
    public Command rollersOn() {
        // Only use YAMS control, not manual rollers.set()
        return run(() -> {
        
            roller.setSpeed(RPM.of(1500));

        }).withName("Rollers On");
    }
    
    public Command rollersOff() {
        return run(() -> {
            roller.setSpeed(RPM.of(0));
        }).withName("Rollers Off");
    }
    
    public Command rollersBackwards() {
        return run(() -> {

            roller.setSpeed(RPM.of(-1500));
        }).withName("Rollers Backwards");
            
        
    }
    
    // Add command methods
    public Command intakeCommand() {
        return runEnd(
            this::rollersOn,
            () -> roller.setSpeed(RPM.of(0))
        );
    }
    
    public Command outtakeCommand() {
        return runEnd(
            this::rollersBackwards,
            () -> roller.setSpeed(RPM.of(0))
        );
    }
    
    
    @Override
    public void simulationPeriodic() {
        // Only simulate, don't manually run the roller
        roller.simIterate();
    }
}