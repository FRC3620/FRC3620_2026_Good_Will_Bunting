package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pound;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

public class ShooterHoodSubsystem extends SubsystemBase{

    TalonFX hood= new TalonFX(58);
    SmartMotorControllerConfig hoodConfig = new SmartMotorControllerConfig(this)
    .withClosedLoopController(4, 0, 0, DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90))
    .withSoftLimit(Degrees.of(-30), Degrees.of(100))
    .withGearing(new MechanismGearing(GearBox.fromReductionStages(7,1)))
    .withIdleMode(MotorMode.BRAKE)
    .withTelemetry("ShooterHoodMotor", TelemetryVerbosity.HIGH)
    .withStatorCurrentLimit(Amps.of(40))
    .withFeedforward(new ArmFeedforward(0, 0, 0, 0))
    .withControlMode(ControlMode.CLOSED_LOOP);
    // .withMOI(Feet.of(4), Pound.of(4));
     private final SmartMotorController hoodmotor = new TalonFXWrapper(hood, DCMotor.getKrakenX60(1),hoodConfig);
     

  private final Arm shooterHood = new Arm(new ArmConfig(hoodmotor)
      .withHardLimit(Degrees.of(45), Degrees.of(90))
      .withStartingPosition(Degrees.of(45))
      .withTelemetry("Shooter Hood", TelemetryVerbosity.HIGH)
       .withLength(Feet.of(1))
       .withMass(Pound.of(0.5))
  );

  @Override
  public void periodic() {
    shooterHood.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    // setAngle(intake /\)
    shooterHood.simIterate();
  }

  public Command setAngle(Angle angle) {
    return shooterHood.setAngle(angle);
  }
}
