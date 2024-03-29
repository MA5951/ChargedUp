// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LED;

import com.ma5951.utils.led.AddressableLEDController;
import com.ma5951.utils.led.BlinkingColorPattern;
import com.ma5951.utils.led.BreathingColorPattern;
import com.ma5951.utils.led.BreathingTripleColorPattern;
import com.ma5951.utils.led.EvenOddColorPattern;
import com.ma5951.utils.led.RainbowColorPatterSimultaneously;
import com.ma5951.utils.led.RainbowColorPattern;
import com.ma5951.utils.led.SolidColorPattern;
import com.ma5951.utils.led.WaveBlinkColorPattern;
import com.ma5951.utils.led.SmoothColorTransitionPattern;
import com.ma5951.utils.led.SmoothWaveColorPattern;
import com.ma5951.utils.led.WavePattern;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PortMap;

public class LED extends SubsystemBase {
  /** Creates a new LEDSubsystem. */
  private static LED led;
  AddressableLEDController ledController;
  SolidColorPattern solidColorPattern;
  RainbowColorPattern rainbowColorPattern;
  RainbowColorPatterSimultaneously rainbowColorPatterSimultaneously;
  BlinkingColorPattern blinkingColorPattern;
  BreathingColorPattern breathingColorPattern;
  BreathingTripleColorPattern breathingTripleColorPattern;
  SmoothColorTransitionPattern smoothColorTransitionPattern;
  WavePattern wavePattern;
  SmoothWaveColorPattern smoothWaveColorPattern;
  WaveBlinkColorPattern waveBlinkColorPattern;
  EvenOddColorPattern evenOddColorPattern;
  public LED() {
    ledController = new AddressableLEDController(PortMap.ledPort, 300);
    solidColorPattern = new SolidColorPattern(Color.kRed);
    rainbowColorPattern = new RainbowColorPattern();
    blinkingColorPattern = new BlinkingColorPattern(Color.kRed, Color.kRed,0);
    breathingColorPattern = new BreathingColorPattern(Color.kRed, 0);
    breathingTripleColorPattern = new BreathingTripleColorPattern(Color.kRed, Color.kBlue, 0);
    rainbowColorPatterSimultaneously = new RainbowColorPatterSimultaneously();
    smoothColorTransitionPattern = new SmoothColorTransitionPattern(Color.kRed, Color.kBlue, 0);
    wavePattern = new WavePattern(2, 5, 1, new Color [] {Color.kRed, Color.kBlue});
    smoothWaveColorPattern = new SmoothWaveColorPattern(2, 5, 1, new Color [] {Color.kRed, Color.kBlue});
    waveBlinkColorPattern = new WaveBlinkColorPattern(Color.kRed, Color.kBlue, 0);
    evenOddColorPattern = new EvenOddColorPattern(Color.kRed, Color.kBlue, 0);
  }

  public void setSolidColor(Color color) {
    solidColorPattern.setColor(color);
    ledController.setAddressableLEDPattern(solidColorPattern);
  }

  public void setSmoothWave(int numColors, double period, double speed, Color[] colors) {
    smoothWaveColorPattern.setParameters(numColors, period, speed, colors);
    ledController.setAddressableLEDPattern(smoothWaveColorPattern);
  }

  public void setRainbow() {
    ledController.setAddressableLEDPattern(rainbowColorPattern);
  }

  public void setSmoothColorTransition(Color color, Color color2, double interval) {
    smoothColorTransitionPattern.setParameters(color, color2, interval);
    ledController.setAddressableLEDPattern(smoothColorTransitionPattern);
  }
  
  public void setFullRainbow() {
    ledController.setAddressableLEDPattern(rainbowColorPatterSimultaneously);
  }


  public void setWave(int numColors, double period, double speed, Color[] colors) {
    wavePattern.setParameters(numColors, period, speed, colors);
    ledController.setAddressableLEDPattern(wavePattern);
  }
  
  public void setEvenOdd(Color color, Color color2, double lenght) {
    evenOddColorPattern.setParameters(color, color2, lenght);
    ledController.setAddressableLEDPattern(evenOddColorPattern);
  }


  public void setWaveBlink(Color color, Color color2, double interval) {
    waveBlinkColorPattern.setParameters(color, color2, interval);
    ledController.setAddressableLEDPattern(waveBlinkColorPattern);
  }


  public void setBlinking(Color color, Color color2, double interval) {
    blinkingColorPattern.setParameters(color, color2, interval);
    ledController.setAddressableLEDPattern(blinkingColorPattern);
  }

  public void setBreathing(Color color, double interval){
    breathingColorPattern.setParameters(color, interval);
    ledController.setAddressableLEDPattern(breathingColorPattern);
  }

  public void setBreathingTriple(Color color, Color color2, double interval){
    breathingTripleColorPattern.setParameters(color, color2, interval);
    ledController.setAddressableLEDPattern(breathingTripleColorPattern);
  }

  public void setAllianceColor() {
    if (DriverStation.isFMSAttached()) {
      if (DriverStation.getAlliance() == Alliance.Red) {
        // setSolidColor(Color.kRed);
        setSmoothWave(2, 1, 1, new Color [] {Constants.ColorPresets.RED, Constants.ColorPresets.BLACK});
      } else if (DriverStation.getAlliance() == Alliance.Blue) {
        // setSolidColor(Color.kBlue);
        setSmoothWave(2, 1, 1, new Color [] {Constants.ColorPresets.BLUE, Constants.ColorPresets.BLACK});
      }
    }
    else{
      setSmoothWave(2, 1, 1, new Color [] {Constants.ColorPresets.CUBE_PURPLE, Constants.ColorPresets.BLACK});
      // setSolidColor(Color.kPurple);
    }
  }

  public static LED getInstance() {
    if (led == null) {
      led = new LED();
    }
    return led;
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled()) {
      setAllianceColor();
    }
    else if (DriverStation.isAutonomous()) {
      setSmoothWave(3, 1, 1, new Color [] {Constants.ColorPresets.CONE_YELLOW, Constants.ColorPresets.CUBE_PURPLE, Constants.ColorPresets.CYAN});
    }
    else if (DriverStation.isEStopped()){
      setWaveBlink(Constants.ColorPresets.RED, Constants.ColorPresets.WHITE, 2);
    }
    else if (DriverStation.isTeleop() && !DriverStation.isJoystickConnected(0)){
      setBlinking(Constants.ColorPresets.RED, Constants.ColorPresets.WHITE, 0.5);
    }
    else if (DriverStation.isTeleop()){
      setSmoothWave(2, 1, 1, new Color [] {Constants.ColorPresets.CONE_YELLOW, Constants.ColorPresets.CUBE_PURPLE});
    }
    //if error in driver station flash leds to red
    // if ()


    // This method will be called once per scheduler run
  }
}
