package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Constants.LEDs.States;
import frc.robot.utils.LED;
import frc.robot.utils.LEDGroup;

public class LEDHandler extends SubsystemBase {
    private LEDGroup ledGroup;

    private int codeLoops = 0;

    private States currentState = States.NOTHING;
    private States desiredState;

    public LEDHandler() {
        ledGroup = new LEDGroup(
            new LED(Constants.LEDs.LEFT_CANDLE_ID),
            new LED(Constants.LEDs.RIGHT_CANDLE_ID)
        );
    }

    public void request(States desiredState) {
        this.desiredState = desiredState;
    }

    public int getCurrentPriority() {
        return currentState.getPriority();
    }

    @Override
    public void periodic() {
        if (currentState != desiredState) {
            if (codeLoops >= Constants.LEDs.CODE_LOOP_THRESHOLD) {
                currentState = desiredState;
                ledGroup.setLEDGroup(currentState);
                codeLoops = 0;
            } else {
                codeLoops++;
            }
        }
    }
}
