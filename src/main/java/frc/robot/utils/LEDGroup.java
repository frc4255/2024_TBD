package frc.robot.utils;

import frc.robot.Constants.LEDs.States;

public class LEDGroup {
    
    private LED[] ledArr;

    public LEDGroup(LED... LEDs) {
        ledArr = LEDs.clone();
    }

    public void setLEDGroup(States desiredState) {
        for (LED led : ledArr) {
            led.updateLEDs(desiredState);
        }
    }
}
