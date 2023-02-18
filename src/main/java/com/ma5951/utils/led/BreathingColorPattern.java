package com.ma5951.utils.led;

import java.util.Timer;
import java.util.TimerTask;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public class BreathingColorPattern implements  AddressableLEDPattern{
    public Color pattern = new Color(0, 0, 0);
    public SolidColorPattern color;
    private double interval;

    public BreathingColorPattern(Color color, double interval) {
        setParameters(color, interval);
    }

    public void setParameters(Color colorOrigen, double interval) {
        this.interval = interval;
    }



    @Override
    public void setLEDs(AddressableLEDBuffer buffer) {
        TimerTask task = new TimerTask() {
            @Override
            public void run() {
                if (pattern.red > 0 || pattern.green > 0 || pattern.blue > 0) {
                    pattern = new Color(pattern.red - 1, pattern.green - 1, pattern.blue - 1);
                    color.setColor(pattern);
                    color.setLEDs(buffer);
                    System.out.println("red: " + pattern.red + ", green: " + pattern.green + ", blue: " + pattern.blue);
                }
            }
        };
        // color.setLEDs(buffer);
        System.out.println("outside: "+"red: " + pattern.red + ", green: " + pattern.green + ", blue: " + pattern.blue);
        Timer timer = new Timer(); 
        timer.scheduleAtFixedRate(task, (long) interval, (long) interval);
    }

    @Override
    public boolean isAnimated() {
        return true;
    }
}
