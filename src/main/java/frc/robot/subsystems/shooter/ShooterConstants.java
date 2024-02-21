package frc.robot.subsystems.shooter;

import java.util.Map;

public class ShooterConstants {

    public static final double PIVOT_P = 0; //TODO

    public static final int PIVOT_MOTOR_ID = 30; //TODO
    public static final double MAX_DISTANCE = 0;
    public static final Map<Double, Double> LOOKUP_TABLE = Map.of();

    public static final double FLYWHEEL_P = 0.1; //TODO

    public static final class Hopper {
        public static final int MOTOR_ID_0 = 20;
        public static final int MOTOR_ID_1 = 21; 
        public static final int ENCODER_ID = 2;
        public static final double HOPPER_CURRENT_THRESHOLD_M0 = 0; // TODO: assign current threshold for motor 0 (star)
        public static final double HOPPER_CURRENT_THRESHOLD_M1 = 0; // TODO: assign current threshold for motor 1 (not the star)
    }
}
