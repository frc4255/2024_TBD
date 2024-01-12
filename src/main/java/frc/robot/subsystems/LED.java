package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import frc.robot.Constants;

//Import candle packaage
public class LED extends LEDHandler {
    //Initialie CANdle
    private CANdle mCandle = new CANdle(Constants.LEDs.CANDLE_ID);

    private States currentState;

    private States futureState;

    public enum States {
        SHOOTING(1, new Color(255,0,0)),
        TARGET_IN_RANGE(2, new Color(0,255,0)),
        NOTE_STORED(3, new Color(255, 128, 0)),
        NOTHING(0, new Color(0,0,0));

        private final int priority;
        private final Color color;

        States(int priority, Color color) {
            this.priority = priority;
            this.color = color;
        }
        
        public int getPriority() {
            return priority;
        }

        public Color getColor() {
            return color;
        }
    }

    public LED() {
        // TODO: Figure this out, currently this will bug out for loops.
        if (currentState.getPriority() < futureState.getPriority()){
            updateLEDs(futureState); 
        }
    }

    public void updateLEDs (States state) {
        mCandle.setLEDs(state.getColor().r, state.getColor().g, state.getColor().b);
    }

    public static class Color {
        public int r;
        public int g;
        public int b;

        public Color() {
            r = 0;
            g = 0;
            b = 0;
        }
        public Color(int RED, int GREEN , int BLUE) {
            r = RED;
            g = GREEN;
            b = BLUE;
            
        }

        public static Color NOTHING(){
            return new Color(0, 0, 0);
        }

        @Override
        public String toString(){
            return "("+r+","+g+","+b+")";
        }
    }
}
