// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.ma5951.utils.MAShuffleboard;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PortMap;

public class IntakePosition extends SubsystemBase {

  private static IntakePosition openIntake;

  private CANSparkMax motor;
  private RelativeEncoder encoder;

  private DigitalInput hallEffect;

  private MAShuffleboard board;
  private String kp = "kp";
  private String ki = "ki";
  private String kd = "kd";

  private SparkMaxPIDController pidController;


  public IntakePosition(){
    motor = new CANSparkMax(
      PortMap.Intake.intakePositionMotorID,
      MotorType.kBrushless);

    encoder = motor.getEncoder();

    hallEffect = new DigitalInput(
      PortMap.Intake.closingHallEffectPort);

    motor.setIdleMode(IdleMode.kCoast);
    encoder.setPositionConversionFactor(
      2 * Math.PI * (1 / IntakeConstants.TICKS_PER_ROUND) * IntakeConstants.GEAR
    );
    board = new MAShuffleboard("IntakePosition");

    // motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 0);

    board.addNum(kp, IntakeConstants.KP);
    board.addNum(ki, IntakeConstants.KI);
    board.addNum(kd, IntakeConstants.KD);

    pidController = motor.getPIDController();
    pidController.setFeedbackDevice(encoder);

    pidController.setP(board.getNum(kp));
    pidController.setI(board.getNum(ki));
    pidController.setD(board.getNum(kd));

    resetEncoder();
  }

  public void calculate(double setPoint) {
    pidController.setReference(
      setPoint, ControlType.kPosition,
      0, Math.cos(
        getPosition()) * IntakeConstants.KG, ArbFFUnits.kPercentOut);
  }

  public void setPower(double power){
    motor.set(power);
  }

  public void resetEncoder(){
    encoder.setPosition(0);
  }

  public double getPosition() {
    return encoder.getPosition();
  }

  public boolean isOpen(){
    return Math.abs(
      encoder.getPosition() - IntakeConstants.CLOSE_POSITION) < 
      IntakeConstants.POSITION_TOLORANCE;
  }

  public boolean isClose(){
    return Math.abs(encoder.getPosition()-IntakeConstants.CLOSE_POSITION) < 
    IntakeConstants.POSITION_TOLORANCE;
  }

  public boolean isMiddle(){
    return Math.abs(encoder.getPosition()-IntakeConstants.MIDDLE_POSITION) < 
    IntakeConstants.POSITION_TOLORANCE;
  }

  public static IntakePosition getInstance() {
    if (openIntake == null) {
      openIntake = new IntakePosition();
    }
    return openIntake;
  }

  @Override
  public void periodic() {
    if (hallEffect.get()) {
      encoder.setPosition(IntakeConstants.CLOSE_POSITION);
    }

    board.addBoolean("isClose", isClose());
    board.addBoolean("isOpen", isOpen());
    board.addBoolean("isMiddle", isMiddle());

    board.addNum("position", encoder.getPosition());

    board.addBoolean("hall effect", hallEffect.get());
  }
}