package frc.robot.subsystems.shooter;

import java.util.Map;

public class ShooterConstants {

    public static final double PIVOT_P = 12;

    public static final int PIVOT_MOTOR_ID = 20;
    public static final double MAX_DISTANCE = 0;
    public static final Map<Double, Double> LOOKUP_TABLE = Map.of();

    public static final double RIGHT_FLYWHEEL_P = 0.002;
    public static final double LEFT_FLYWHEEL_P = 0.0018;

    public static final class Hopper {
        public static final int HOPPER_COMPLIANT_ID = 30;
        public static final int HOPPER_STAR_ID = 31; 
        public static final int ENCODER_ID = 2;
    }
}
