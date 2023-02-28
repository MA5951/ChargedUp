// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Spinner;

import com.ma5951.utils.MAShuffleboard;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PortMap;

public class Spinner extends SubsystemBase {
  private static Spinner spinnerSubsystem;

  private CANSparkMax spinnerMotor;
  
  private DigitalInput buttomIR;

  private RelativeEncoder encoder;

  private MAShuffleboard board;

  private Spinner() {
    spinnerMotor = new CANSparkMax(
      PortMap.Spinner.spinnerMotorID, MotorType.kBrushless);
    buttomIR = new DigitalInput(PortMap.Spinner.buttomIRPort);

    encoder = spinnerMotor.getEncoder();

    encoder.setPositionConversionFactor(
      SpinnerConstants.POSITION_CONVERSION_FACTOR);
    board = new MAShuffleboard("Spinner");

    encoder.setPosition(0);
  }

  public boolean isGamePiceEntered() {
    return !buttomIR.get();
  }

  public void setPower(double power){
    spinnerMotor.set(power);
  }

  public double getPosition(){
    return encoder.getPosition();
  }

  public void resetEncoder(){
    encoder.setPosition(0);
  }

  public double getVoltage(){
    return spinnerMotor.getBusVoltage();
  }

  public static Spinner getInstance() {
    if (spinnerSubsystem == null) {
      spinnerSubsystem = new Spinner();
    }
    return spinnerSubsystem;
  }

  @Override
  public void periodic() {
    board.addBoolean("isGamePiceEntered", isGamePiceEntered());
    board.addNum("position", getPosition());
  }
}