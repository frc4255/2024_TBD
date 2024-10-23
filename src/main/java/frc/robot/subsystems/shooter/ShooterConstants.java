package frc.robot.subsystems.shooter;

import java.util.Map;

public class ShooterConstants {

    public static final double PIVOT_P = 17;

    public static final int PIVOT_MOTOR_ID = 20;
    public static final double MAX_DISTANCE = 7;
    public static final double MIN_DISTANCE = 1.5;
    /*
     * Lookup table for pivot interpolation table. Key is distance (m), value is angle (radians)
     */
    public static final Map<Double, Double> LOOKUP_TABLE = Map.ofEntries(
        Map.entry(1.5, 0.62),
        Map.entry(2.0, 0.61),
        Map.entry(2.3, 0.56),
        Map.entry(2.5, 0.48),
        Map.entry(3.0, 0.4),
        Map.entry(3.5, 0.38),
        Map.entry(4.0, 0.37),
        Map.entry(4.5, 0.35),
        Map.entry(5.0, 0.33),
        Map.entry(5.3, 0.3),
        Map.entry(5.5, 0.28),
        Map.entry(6.0, 0.265),
        Map.entry(6.5, 0.26),
        Map.entry(7.0, 0.255)
    );

    public static final double RIGHT_FLYWHEEL_P = 0.0022;
    public static final double LEFT_FLYWHEEL_P = 0.0022;

    public static final class Hopper {
        public static final int HOPPER_COMPLIANT_ID = 30;
        public static final int HOPPER_STAR_ID = 31; 
        public static final int ENCODER_ID = 2;
    }
}
