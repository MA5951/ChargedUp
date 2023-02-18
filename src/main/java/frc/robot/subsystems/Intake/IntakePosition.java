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
      2 * Math.PI * (1 / IntakeConstants.ticksPerRound) * IntakeConstants.gear
    );

    board.addNum(kp, IntakeConstants.kp);
    board.addNum(ki, IntakeConstants.ki);
    board.addNum(kd, IntakeConstants.kd);

    pidController = motor.getPIDController();
    pidController.setFeedbackDevice(encoder);

    pidController.setP(board.getNum(kp));
    pidController.setI(board.getNum(ki));
    pidController.setD(board.getNum(kd));

    board = new MAShuffleboard("IntakePosition");
    resetEncoder();
  }

  public void calculate(double setPoint) {
    pidController.setReference(
      setPoint, ControlType.kPosition,
      0, Math.cos(
        getPosition()) * IntakeConstants.kG, ArbFFUnits.kPercentOut);
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
      encoder.getPosition() - IntakeConstants.ClosePosition) < 
      IntakeConstants.positionTolorance;
  }

  public boolean isClose(){
    return Math.abs(encoder.getPosition()-IntakeConstants.ClosePosition) < 
    IntakeConstants.positionTolorance;
  }

  public boolean isMiddle(){
    return Math.abs(encoder.getPosition()-IntakeConstants.MiddlePosition) < 
    IntakeConstants.positionTolorance;
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
      encoder.setPosition(IntakeConstants.ClosePosition);
    }
    pidController.setP(board.getNum(kp));
    pidController.setI(board.getNum(ki));
    pidController.setD(board.getNum(kd));

    board.addBoolean("isClose", isClose());
    board.addBoolean("isOpen", isOpen());
    board.addBoolean("isMiddle", isMiddle());

    board.addNum("position", encoder.getPosition());
  }
}