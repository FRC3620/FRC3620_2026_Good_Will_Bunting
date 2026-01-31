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
public class ConvaderSubsystem extends SubsystemBase {
    
    private  TalonFX motor=null;
    private  SmartMotorController motorController;
    private  FlyWheel flyWheel;
    
    public ConvaderSubsystem() {
        boolean makeDevices= RobotContainer.canDeviceFinder.isDevicePresent(org.usfirst.frc3620.CANDeviceType.TALON_PHOENIX6, 
        Constants.MOTORID_CONVADER
        , "Convader Rollers")||RobotContainer.shouldMakeAllCANDevices();
        
        if(makeDevices){
            motor = new TalonFX(Constants.MOTORID_CONVADER);

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
                .withTelemetry("ConvaderMotor", TelemetryVerbosity.HIGH)
                .withStatorCurrentLimit(Amps.of(40))
                .withSupplyCurrentLimit(Amps.of(40))
                .withControlMode(ControlMode.CLOSED_LOOP);
            
                motorController = new TalonFXWrapper(motor, DCMotor.getKrakenX60(1), motorConfig);
                FlyWheelConfig rollerConfig = new FlyWheelConfig(motorController)
                    .withDiameter(Inch.of(4))
                    .withMass(Pound.of(0.5))
                    .withUpperSoftLimit(RPM.of(7000))
                    .withTelemetry("Convader Roller", TelemetryVerbosity.HIGH);
                
                // Create the FlyWheel
                flyWheel = new FlyWheel(rollerConfig);
        }
    }
    public Command setSpeed(double speed) {

            if (flyWheel != null) {
               return flyWheel.setSpeed(RPM.of(speed));
            } else {
                return null;
            }
    }
    
    @Override
    public void simulationPeriodic() {
        // Only simulate, don't manually run the roller
        flyWheel.simIterate();
    }
}
