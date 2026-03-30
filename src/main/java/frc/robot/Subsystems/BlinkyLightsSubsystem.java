// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import org.usfirst.frc3620.RobotMode;

import com.fasterxml.jackson.databind.deser.DataFormatReaders.Match;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Helpers.FMSTriggers;
import frc.robot.Subsystems.HealthSubsystem.Health;
import frc.robot.fsm.states.IState;
import frc.robot.fsm.states.ScoringState;

public class BlinkyLightsSubsystem extends SubsystemBase {

  int length = 38;

  AddressableLED m_led = new AddressableLED(9);

  AddressableLEDBuffer m_buffer = new AddressableLEDBuffer(length);

  Distance ledSpacing = Meters.of(1 / 120.0);
  LEDPattern base = LEDPattern.solid(Color.kRed);
  LEDPattern driverDisabled = LEDPattern.solid(Color.kOrange);
  LEDPattern driverTeleop = LEDPattern.solid(Color.kGreen);
  LEDPattern good = LEDPattern.solid(Color.kGreen);
  LEDPattern mediocre = LEDPattern.solid(Color.kYellow);
  LEDPattern bad = LEDPattern.solid(Color.kRed);
  LEDPattern deathrow = base.blink(Seconds.of(0.5));// will also be used for turret wrapping point as well as health
                                                    // subsystem.

  LEDPattern reallyCloseToWrapping = base.blink(Seconds.of(0.1));

  AddressableLEDBufferView m_healthLeft = m_buffer.createView(0, 4);
  AddressableLEDBufferView m_healthRight = m_buffer.createView(34, length - 1);

  AddressableLEDBufferView m_wrappingRight = m_buffer.createView(5, 9);
  AddressableLEDBufferView m_wrappingLeft = m_buffer.createView(29, 33);

  AddressableLEDBufferView m_driver = m_buffer.createView(10, 28);

  /** Creates a new BlinkyLightsSubsystem. */
  public BlinkyLightsSubsystem() {
    m_led.setLength(length);
    m_led.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Create an LED pattern that sets the entire strip to solid red
    Health currentHealth = RobotContainer.healthSubsystem.getCurrentHealth();
    LEDPattern healthCurrentPattern = good;
    LEDPattern driverCurrentPattern = good;

    LEDPattern leftWrapping = good;
    LEDPattern rightWrapping = good;

    TurretSubsystem turret = RobotContainer.turretSubsystem;

    if (currentHealth == Health.MEDIOCRE) {
      healthCurrentPattern = mediocre;
    } else if (currentHealth == Health.BAD) {
      healthCurrentPattern = bad;
    } else if (currentHealth == Health.DEATHROW) {
      healthCurrentPattern = deathrow;
    }
    // Apply the LED pattern to the data buffer
    healthCurrentPattern.applyTo(m_healthLeft);
    healthCurrentPattern.applyTo(m_healthRight);

    RobotMode robotMode = Robot.getCurrentRobotMode();
    IState currentState = RobotContainer.getStateMachine().getCurrentState();

    if (robotMode == RobotMode.DISABLED) {
      driverCurrentPattern = driverDisabled;
    } else {
      driverCurrentPattern = currentState.getLEDPattern();
    }

    if (turret.isReallyCloseToLeftWrapping()) {
      leftWrapping = reallyCloseToWrapping;
      rightWrapping = bad;

    } else if (turret.isNearLeftWrapping()) {
      leftWrapping = deathrow;
      rightWrapping = bad;

    } else if (turret.isReallyCloseToRightWrapping()) {
      rightWrapping = reallyCloseToWrapping;
      leftWrapping = bad;

    } else if (turret.isNearRightWrapping()) {
      rightWrapping = deathrow;
      leftWrapping = bad;

    } else {
      rightWrapping = good;
      leftWrapping = good;
    }

    leftWrapping.applyTo(m_wrappingLeft);
    rightWrapping.applyTo(m_wrappingRight);

    double matchTime = Timer.getMatchTime();
    SmartDashboard.putNumber("Match Time", matchTime);
    if (RobotContainer.useFMSTriggers.getAsBoolean() == true) {
      if ((matchTime <= 115 && matchTime > 110) || (matchTime <= 90 && matchTime > 85) ||
          (matchTime <= 65 && matchTime > 60) || (matchTime <= 40 && matchTime > 35)) {
        driverCurrentPattern = driverCurrentPattern.blink(Seconds.of(0.4));
      } else if ((matchTime <= 110 && matchTime > 105) || (matchTime <= 85 && matchTime > 80) ||
          (matchTime <= 60 && matchTime > 55) || (matchTime <= 35 && matchTime > 30)) {
        driverCurrentPattern = driverCurrentPattern.blink(Seconds.of(0.1));
      }
    } else {
    }
    driverCurrentPattern.applyTo(m_driver);

    // Write the data to the LED strip
    m_led.setData(m_buffer);
  }
}
