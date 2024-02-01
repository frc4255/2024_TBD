package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.ColorSensorV3.ProximitySensorMeasurementRate;
import com.revrobotics.ColorSensorV3.ProximitySensorResolution;

public final class Constants {
    public static class Hopper {
        public static final int MOTOR_ID_0 = 0;
        public static final int MOTOR_ID_1 = 1; 
        public static final int ENCODER_ID = 2;
        public static final double HOPPER_CURRENT_THRESHOLD_M0 = 0; // TODO: assign current threshold for motor 0 (star)
        public static final double HOPPER_CURRENT_THRESHOLD_M1 = 0; // TODO: assign current threshold for motor 1 (not the star)
    }
}