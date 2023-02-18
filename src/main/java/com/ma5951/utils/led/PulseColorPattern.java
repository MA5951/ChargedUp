package com.ma5951.utils.led;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;

public class PulseColorPattern implements AddressableLEDPattern {
    private double red;
    private double green;
    private double blue;
    private double redOrigin;
    private double greenOrigin;
    private double blueOrigin;
    private double interval;
    private double timestamp;

    public PulseColorPattern(Color color, double interval) {
        red = 0;
        green = 0;
        blue = 0;
        setParameters(color, interval);
    }

    public void setParameters(Color color, double interval) {
        red = color.red;
        green = color.green;
        blue = color.blue;
        redOrigin = color.red;
        greenOrigin = color.green;
        blueOrigin = color.blue;
        this.interval = interval;
    }   

    @Override
    public void setLEDs(AddressableLEDBuffer buffer) {
        double elapsed = Timer.getFPGATimestamp() - timestamp;
        double pulse = Math.sin(elapsed / interval * Math.PI) * 0.5 + 0.5;
        int newRed = (int) Math.round(red * (1 - pulse) + redOrigin * pulse);
        int newGreen = (int) Math.round(green * (1 - pulse) + greenOrigin * pulse);
        int newBlue = (int) Math.round(blue * (1 - pulse) + blueOrigin * pulse);
        newRed = Math.min(255, Math.max(0, newRed));
        newGreen = Math.min(255, Math.max(0, newGreen));
        newBlue = Math.min(255, Math.max(0, newBlue));
        Color color = new Color(newRed / 255.0, newGreen / 255.0, newBlue / 255.0);
        SolidColorPattern pattern = new SolidColorPattern(color);
        pattern.setLEDs(buffer);
        red = newRed / 255.0;
        green = newGreen / 255.0;
        blue = newBlue / 255.0;
        System.out.println("red: " + red + ", green: " + green + ", blue: " + blue);
        System.out.println("elapsed: " + elapsed + ", pulse: " + pulse);
        System.out.println("newRed: " + newRed + ", newGreen: " + newGreen + ", newBlue: " + newBlue);
    }

    @Override
    public boolean isAnimated() {
        return true;
    }
}
