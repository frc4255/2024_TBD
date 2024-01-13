package frc.robot.utils;

import com.ctre.phoenix.led.CANdle;
import frc.robot.Constants;

public class LED {
    private CANdle mCandle;

    public LED(int ID) {
        mCandle = new CANdle(ID);
    }

    public void updateLEDs (Constants.LEDs.States state) {
        mCandle.setLEDs(state.getColor().r, state.getColor().g, state.getColor().b);
    }
}
