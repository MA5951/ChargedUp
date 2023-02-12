// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Spinner;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Spinner extends SubsystemBase {
  // create subsystem
  private static Spinner spinnerSubsystem;
  //create spinner motor
  private CANSparkMax spinnerMotor;
  //create spinner ir sensor (digital input)
  private DigitalInput buttomIR;
  private DigitalInput stuckIR;

  private RelativeEncoder encoder;

  //create spinner motor and ir sensor
  public Spinner() {
    //initialize spinner motor
    spinnerMotor = new CANSparkMax(SpinnerPortMap.motorID, MotorType.kBrushless);
    //initialize spinner ir sensor
    buttomIR = new DigitalInput(SpinnerPortMap.buttomIRChanlle);
    stuckIR = new DigitalInput(SpinnerPortMap.stuckIRChanlle);

    encoder = spinnerMotor.getEncoder();

    encoder.setPositionConversionFactor(360*(1/SpinnerConstants.ticksPerRound));
  }

  // get sensor value
  public boolean isGamePiceEntered() {
    return buttomIR.get();
  }

  public boolean isStuck() {
    return stuckIR.get();
  }

  // if sensor is true, spin motor in one direction (clockwise) if false, spin motor in other direction (counter clockwise) for 1 spin (42 motor ticks)
  public void setVoltage(double voltage) {
    spinnerMotor.setVoltage(voltage);
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

  // create subsystem method
  public static Spinner getInstance() {
    if (spinnerSubsystem == null) {
      spinnerSubsystem = new Spinner();
    }
    return spinnerSubsystem;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
