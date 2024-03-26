package frc.robot.utils;

import com.ctre.phoenix.led.Animation;

public class Color {
    public int r;
    public int g;
    public int b;
    public boolean strobe;
    public double animationSpeed;
    
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

    public Color(int RED, int GREEN , int BLUE, boolean strobe, double speed) {
        r = RED;
        g = GREEN;
        b = BLUE;
        this.strobe = strobe;
        this.animationSpeed = speed;
    }
}