package frc.robot.subsystems;

package com.team1678.frc2023.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdleControlFrame;
import com.ctre.phoenix.led.CANdleStatusFrame;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LEDHandler {

    private final CANdle mCandle = new CANdle(Ports.CANDLE, "canivore1");

    private static LEDs mInstance;

    private final int kNumLeds = 59;

    public static LEDs getInstance() {
        if (mInstance == null) {
            mInstance = new LEDs();
        }
        return mInstance;
    }
    
    public enum States {
        SHOOTING,
        TARGET_IN_RANGE,
        NOTE_STORED,
        NOTHING

    }
}
