// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;

public class LEDSubsystem extends SubsystemBase {
    private final CANdle m_candle = new CANdle(26);
    private final int LedCount = 121; 
    private boolean CoralisReadyPast = false;
 
    public LEDSubsystem() {
        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = false;
        configAll.disableWhenLOS = false;
        configAll.stripType = LEDStripType.GRB;
        configAll.brightnessScalar = 0.05;
        configAll.vBatOutputMode = VBatOutputMode.Modulated;
        m_candle.configAllSettings(configAll, 100);

        startup();
    }

  

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // Only change LEDs if needed.
        if (constants.k_CoralisReady && !CoralisReadyPast) {
            green();
            CoralisReadyPast = constants.k_CoralisReady;
        } else if (!constants.k_CoralisReady && CoralisReadyPast) {
            pink();
            CoralisReadyPast = constants.k_CoralisReady;
        } else {
            CoralisReadyPast = constants.k_CoralisReady;
        }
    }


    public void red() {
        m_candle.setLEDs(255, 0, 0, 0, 0, LedCount);
    }
    
    public void pink() {
        m_candle.setLEDs(255, 97, 171, 0, 0, LedCount);
    }

    public void green() {
        m_candle.setLEDs(0, 255, 0, 0, 0, LedCount);
    }

    public void startup() {
        m_candle.setLEDs(255, 97, 171, 0, 0, LedCount);
    }

}
