// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Spinner;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Spinner extends SubsystemBase {
  // create subsystem
  private static Spinner spinnerSubsystem;
  //create spinner motor
  private CANSparkMax spinnerMotor;
  //create spinner ir sensor (digital input)
  private DigitalInput spinnerIR;

  //create spinner motor and ir sensor
  public Spinner() {
    //initialize spinner motor
    spinnerMotor = new CANSparkMax(0, MotorType.kBrushless);
    //initialize spinner ir sensor
    spinnerIR = new DigitalInput(0);
  }

  // get sensor value
  public boolean getIR() {
    return spinnerIR.get();
  }

  // get motor encoder value
  public double getEncoder() {
    return spinnerMotor.getEncoder().getPosition();
  }

  // if sensor is true, spin motor in one direction (clockwise) if false, spin motor in other direction (counter clockwise) for 1 spin (42 motor ticks)
  public void setVoltage(double voltage) {
    spinnerMotor.setVoltage(voltage);
  }

  public void setVelocity(double velocity){
    spinnerMotor.set(velocity);
  }

  //TODO: add setPower function to set the motor via precentage input

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
