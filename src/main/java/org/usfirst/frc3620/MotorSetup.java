// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.usfirst.frc3620;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

/** Add your docs here. */
@SuppressWarnings("unused")
public class MotorSetup {
    public static void resetMaxToKnownState(SparkMaxSendable x, boolean inverted) {
        SparkMaxConfig config = new SparkMaxConfig();
        // TODO set to factory default 
        config.inverted(inverted);

        config.idleMode(IdleMode.kCoast);
        config.openLoopRampRate(1);
        config.closedLoopRampRate(1);
        config.smartCurrentLimit(80);
    }
    
    public static void resetTalonFXToKnownState(TalonFX m, InvertedValue invert) {
        int kTimeoutMs = 0;

        TalonFXConfiguration config = new TalonFXConfiguration();
        // TODO make this work
        // m.configFactoryDefault();
        config.MotorOutput.Inverted = invert;

        /*
   
        //set max and minimum(nominal) speed in percentage output
        m.configNominalOutputForward(+1, kTimeoutMs);
        m.configNominalOutputReverse(-1, kTimeoutMs);
        m.configPeakOutputForward(+1, kTimeoutMs);
        m.configPeakOutputReverse(-1, kTimeoutMs);
        
        StatorCurrentLimitConfiguration amperage=new StatorCurrentLimitConfiguration(true,40,0,0);
        m.configStatorCurrentLimit(amperage);
        */
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        m.getConfigurator().apply(config);
    }
    
}