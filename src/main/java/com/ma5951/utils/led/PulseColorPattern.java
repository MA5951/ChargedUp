package com.ma5951.utils.led;

import java.io.Console;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;

public class PulseColorPattern implements  AddressableLEDPattern{
    private SolidColorPattern onPattern;
    private SolidColorPattern offPattern;
    private SolidColorPattern curPattern;
    private double interval;
    private boolean on;
    private double lastChange;
    private double red;
    private double green;
    private double blue;
    private double colorAvg;

    public PulseColorPattern(Color onColor, double interval) {
        onPattern = new SolidColorPattern(onColor);
        offPattern = new SolidColorPattern(Color.kBlack);
        red = onColor.red;
        green = onColor.green;
        blue = onColor.blue;
        this.interval = interval;
    }

    public void setInterval(double interval) {
        this.interval = interval;
    }


    @Override
    public void setLEDs(AddressableLEDBuffer buffer) {
        colorAvg = (red + green + blue) / 3;
        curPattern = new SolidColorPattern(new Color(0, 0, 0));
        for (double i = 0; i <= colorAvg; i = i + (colorAvg / interval)){
            curPattern.setColor(new Color(red / i, green / i, blue / i)); 
            curPattern.setLEDs(buffer);
        }
    }

    @Override
    public boolean isAnimated() {
        return true;
    }
}
