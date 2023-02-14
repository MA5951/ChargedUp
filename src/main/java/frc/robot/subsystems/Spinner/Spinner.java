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
import frc.robot.Constants;

public class Spinner extends SubsystemBase {
  private static Spinner spinnerSubsystem;

  private CANSparkMax spinnerMotor;
  
  private DigitalInput buttomIR;
  private DigitalInput stuckIR;

  private RelativeEncoder encoder;

  private MAShuffleboard board;

  public Spinner() {
    spinnerMotor = new CANSparkMax(
      Constants.PortMap.SpinnerPortMap.motorID, MotorType.kBrushless);
    buttomIR = new DigitalInput(Constants.PortMap.SpinnerPortMap.buttomIRChanlle);
    stuckIR = new DigitalInput(Constants.PortMap.SpinnerPortMap.stuckIRChanlle);

    encoder = spinnerMotor.getEncoder();

    encoder.setPositionConversionFactor(360*(1/SpinnerConstants.ticksPerRound));
    board = new MAShuffleboard("Spinner");
  }

  public boolean isGamePiceEntered() {
    return buttomIR.get();
  }

  public boolean isStuck() {
    return stuckIR.get();
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
    board.addBoolean("isStuck", isStuck());
    board.addNum("position", getPosition());
  }
}