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
        Map.entry(1.5, 0.67),
        Map.entry(2.0, 0.63),
        Map.entry(2.3, 0.51),
        Map.entry(2.5, 0.45),
        Map.entry(3.0, 0.39),
        Map.entry(3.5, 0.33),
        Map.entry(4.0, 0.3),
        Map.entry(4.5, 0.27),
        Map.entry(5.0, 0.23),
        Map.entry(5.3, 0.215),
        Map.entry(5.5, 0.21),
        Map.entry(6.0, 0.2),
        Map.entry(6.5, 0.105),
        Map.entry(7.0, 0.10)
    );

    public static final double RIGHT_FLYWHEEL_P = 0.002;
    public static final double LEFT_FLYWHEEL_P = 0.0018;

    public static final class Hopper {
        public static final int HOPPER_COMPLIANT_ID = 30;
        public static final int HOPPER_STAR_ID = 31; 
        public static final int ENCODER_ID = 2;
    }
}
