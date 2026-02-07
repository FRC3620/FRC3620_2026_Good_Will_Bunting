// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Map;

import org.usfirst.frc3620.RobotMode;

import com.ctre.phoenix6.controls.RainbowAnimation;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class BlinkyLightsSubsystem extends SubsystemBase {
  int length = 38;

  AddressableLED m_led = new AddressableLED(9);

  AddressableLEDBuffer m_buffer = new AddressableLEDBuffer(length);

  Distance ledSpacing = Meters.of(1 / 120.0);
  LEDPattern base = LEDPattern.solid(Color.kRed);
  LEDPattern driverDisabled = LEDPattern.solid(Color.kOrange);
  LEDPattern driverTeleop = LEDPattern.solid(Color.kGreen);
  LEDPattern critical = base.blink(Seconds.of(0.5));

  AddressableLEDBufferView m_health = m_buffer.createView(0, length / 2);
  AddressableLEDBufferView m_driver = m_buffer.createView(length / 2 + 1, length - 1).reversed();

  /** Creates a new BlinkyLightsSubsystem. */
  public BlinkyLightsSubsystem() {
    m_led.setLength(length);
    m_led.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Create an LED pattern that sets the entire strip to solid red

    // Apply the LED pattern to the data buffer
    critical.applyTo(m_health);

    RobotMode robotMode = Robot.getCurrentRobotMode();
    if (robotMode == RobotMode.TELEOP) {
      driverTeleop.applyTo(m_driver);
    } 
    if (robotMode == RobotMode.DISABLED) {
      driverDisabled.applyTo(m_driver);
    }

    // Write the data to the LED strip
    m_led.setData(m_buffer);
  }
}
