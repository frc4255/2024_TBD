package frc.robot.subsystems;

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

import frc.robot.Constants;

public class LEDHandler {

    private final CANdle mCandle = new CANdle(Constants.LEDs.CANDLE_ID);

    private static LED mInstance;

    public enum States {
        SHOOTING(1),
        TARGET_IN_RANGE(2),
        NOTE_STORED(3),
        NOTHING(0);

        private final int priority;

        States(int priority) {
            this.priority = priority;
        }
    }
}
