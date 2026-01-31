package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Feet;
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
import frc.robot.RobotContainer;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXSWrapper;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class IntakeShoulderSubsystem extends SubsystemBase{
    public boolean makeDevice = RobotContainer.canDeviceFinder.isDevicePresent(CANDeviceType.CANCODER_PHOENIX6, 
    Constants.MOTORID_INTAKE_SHOULDER)|| RobotContainer.shouldMakeAllCANDevices();
    TalonFX motor= null;
    SmartMotorControllerConfig motorConfig =null;
    private  SmartMotorController motorControler =null;
    private  Arm arm = null;
    public enum IntakeShoulderPositions{
        UP(90.0),
        Down(0.0);
        IntakeShoulderPositions(Double i) {
            //TODO Auto-generated constructor stub
        }}
     


  public IntakeShoulderSubsystem() {
    if(makeDevice){
      motor= new TalonFX(Constants.MOTORID_INTAKE_SHOULDER);
      motorConfig = new SmartMotorControllerConfig(this)
        .withClosedLoopController(4, 0, 0, DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90))
        .withSoftLimit(Degrees.of(-30), Degrees.of(100))
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(70,1)))
        .withIdleMode(MotorMode.BRAKE)
        .withTelemetry("ShoulderMotor", TelemetryVerbosity.HIGH)
        .withStatorCurrentLimit(Amps.of(40))
        .withFeedforward(new ArmFeedforward(0, 0, 0, 0))
        .withControlMode(ControlMode.CLOSED_LOOP);

        motorControler =  new TalonFXWrapper(motor, DCMotor.getKrakenX60(1),motorConfig);
        arm=new Arm(new ArmConfig(motorControler)
      .withLength(Meters.of(0.135))
      .withHardLimit(Degrees.of(-100), Degrees.of(200))
      .withStartingPosition(Degrees.of(0))
      .withTelemetry("Shoulder Example", TelemetryVerbosity.HIGH)
       .withLength(Feet.of(4))
       .withMass(Pound.of(4))
  );
    }
  }
  @Override
  public void periodic() {
    arm.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    // setAngle(intake /\)
    arm.simIterate();
  }

  public Command setAngle(Angle angle) {
    return arm.setAngle(angle);
  }
}
