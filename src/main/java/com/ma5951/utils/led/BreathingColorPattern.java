package com.ma5951.utils.led;

// import java.util.Timer;
// import java.util.TimerTask;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public class BreathingColorPattern implements  AddressableLEDPattern{
    public Color pattern = new Color(0, 0, 0);
    public SolidColorPattern color;
    // private static Timer timer = new Timer("ledTimer");
    private double interval;
    private double lastChange;
    private double timestamp;
    private boolean direction;

    private double originalRed;
    private double originalGreen;
    private double originalBlue;
    private double maxColor;

    private int cycle = 0;

    public BreathingColorPattern(Color color, double interval) {
        setParameters(color, interval);
        this.color = new SolidColorPattern(color);
    }

    public void setParameters(Color color, double interval) {
        this.interval = interval;
        originalBlue = color.blue;
        originalGreen = color.green;
        originalRed = color.red;
        color = new Color(color.red, color.green, color.blue);
        pattern = new Color(color.red, color.green, color.blue);
        maxColor = Math.max(Math.max(originalBlue, originalGreen), originalRed);
        lastChange = Timer.getFPGATimestamp();
        direction = true;
        // color = new Color(color.red, color.green, color.blue);
    }



    @Override
    public void setLEDs(AddressableLEDBuffer buffer) {
        // timestamp = Timer.getFPGATimestamp();
        // if (timestamp - lastChange > 0) {
        if (direction) {
            if (pattern.red > 0) {
                pattern = new Color(pattern.red - (originalRed / (interval/0.05)), pattern.green, pattern.blue); // (interval/0.05/255.0)
            }
            if (pattern.green > 0) {
                pattern = new Color(pattern.red, pattern.green - (originalGreen / (interval/0.05)), pattern.blue);
            }
            if (pattern.blue > 0) {
                pattern = new Color(pattern.red, pattern.green, pattern.blue - (originalBlue / (interval/0.05)));
            }
            if (pattern.red <= 4.9E-4 && pattern.green <= 4.9E-4 && pattern.blue <= 4.9E-4) {
                direction = false;
            }
            color = new SolidColorPattern(pattern);
        }
        else {
            if (pattern.red < originalRed) {
                pattern = new Color(pattern.red + (originalRed / (interval/0.05)), pattern.green, pattern.blue); // TODO: switch 255 to original color devided by the maxColor
            }
            if (pattern.green < originalGreen) {
                pattern = new Color(pattern.red, pattern.green + (originalGreen / (interval/0.05)), pattern.blue);
            }
            if (pattern.blue < originalBlue) {
                pattern = new Color(pattern.red, pattern.green, pattern.blue + (originalBlue / (interval/0.05)));
            }
            if (pattern.red >= originalRed && pattern.green >= originalGreen && pattern.blue >= originalBlue) {
                direction = true;
            }
            color = new SolidColorPattern(pattern);
        }
        cycle++;
            // color = new SolidColorPattern(pattern);
            // lastChange = timestamp;
        // }
        color.setLEDs(buffer);
        System.out.println("cycle: " + cycle + ", red: " + pattern.red + ", green: " + pattern.green + ", blue: " + pattern.blue);
        System.out.println("timestamp: " + timestamp + ", lastChange: " + lastChange + ", interval: " + interval);
        System.out.println("direction: " + direction);
        System.out.println("originalRed: " + originalRed + ", originalGreen: " + originalGreen + ", originalBlue: " + originalBlue);
    }

    @Override
    public boolean isAnimated() {
        return true;
    }
}