// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BlinkyLightsSubsystem extends SubsystemBase {
  AddressableLED m_led = new AddressableLED(9);

  AddressableLEDBuffer m_buffer = new AddressableLEDBuffer(40);
   LEDPattern pattern = LEDPattern.rainbow(255, 200);
   private static final Distance kLedSpacing = Meters.of(1 / 120.0);
  private final LEDPattern patternrainbowscroll = pattern.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), kLedSpacing);

  /** Creates a new BlinkyLightsSubsystem. */
  public BlinkyLightsSubsystem() {
    m_led.setLength(40);
    m_led.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Create an LED pattern that sets the entire strip to solid red
   
    // Apply the LED pattern to the data buffer
    patternrainbowscroll.applyTo(m_buffer);

    // Write the data to the LED strip
    m_led.setData(m_buffer);
  }
}
