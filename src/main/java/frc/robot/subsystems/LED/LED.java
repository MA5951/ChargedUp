// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LED;

import com.ma5951.utils.led.AddressableLEDController;
import com.ma5951.utils.led.BlinkingColorPattern;
import com.ma5951.utils.led.RainbowColorPattern;
import com.ma5951.utils.led.SolidColorPattern;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PortMap;

public class LED extends SubsystemBase {
  /** Creates a new LEDSubsystem. */
  private static LED led;
  AddressableLEDController ledController;
  SolidColorPattern solidColorPattern;
  RainbowColorPattern rainbowColorPattern;
  BlinkingColorPattern blinkingColorPattern;
  public LED() {
    ledController = new AddressableLEDController(PortMap.ledPort, 300);
    solidColorPattern = new SolidColorPattern(Color.kRed);
    rainbowColorPattern = new RainbowColorPattern();
    blinkingColorPattern = new BlinkingColorPattern(Color.kRed, 0);
  }

  public void setSolidColor(Color color) {
    solidColorPattern.setColor(color);
    ledController.setAddressableLEDPattern(solidColorPattern);
  }

  public void setRainbow() {
    ledController.setAddressableLEDPattern(rainbowColorPattern);
  }

  public void setBlinking(Color color, double interval) {
    blinkingColorPattern.setInterval(interval);
    ledController.setAddressableLEDPattern(blinkingColorPattern);
  }

  public void setAllianceColor() {
    if (DriverStation.isFMSAttached()) {
      if (DriverStation.getAlliance() == Alliance.Red) {
        setSolidColor(Color.kRed);
      } else {
        setSolidColor(Color.kBlue);
      }
    }
    setSolidColor(Color.kPurple);
  }

  public static LED getInstance() {
    if (led == null) {
      led = new LED();
    }
    return led;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
