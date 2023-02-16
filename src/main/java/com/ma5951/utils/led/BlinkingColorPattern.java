package com.ma5951.utils.led;

import java.io.Console;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;

public class BlinkingColorPattern implements  AddressableLEDPattern{
    private SolidColorPattern onPattern;
    private SolidColorPattern offPattern;
    private double interval;
    private boolean on;
    private double lastChange;

    public BlinkingColorPattern(Color onColor, Color offColor, double interval) {
        onPattern = new SolidColorPattern(onColor);
        offPattern = new SolidColorPattern(offColor);
        this.interval = interval;
    }

    public void setInterval(double interval) {
        this.interval = interval;
    }


    @Override
    public void setLEDs(AddressableLEDBuffer buffer) {
        double timestamp = Timer.getFPGATimestamp();
        if (timestamp - lastChange > interval) {
            on = !on;
            lastChange = timestamp;
        }
        if (on) {
            onPattern.setLEDs(buffer);
        }
        else {
            offPattern.setLEDs(buffer);
        }
    }

    @Override
    public boolean isAnimated() {
        return true;
    }
}
