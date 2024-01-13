package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;


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
//add led now