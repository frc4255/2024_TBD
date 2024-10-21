package frc.robot;

public class StateManager {
    public enum RobotStateMachine {
        NORMAL,
        AMP,
        A10BRRRRR,
        SHUTTLE
    }

    private RobotStateMachine currentState = RobotStateMachine.NORMAL;

    public StateManager() {
    }

    public RobotStateMachine getCurrentState() {
        return currentState;
    }

    public void setRobotState(RobotStateMachine newState) {
        currentState = newState;
    }
}
