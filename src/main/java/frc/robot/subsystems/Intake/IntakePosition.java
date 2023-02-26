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
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmRotation;

public class IntakePosition extends SubsystemBase {

  private static IntakePosition openIntake;

  private CANSparkMax motor;
  private RelativeEncoder encoder;

  private DigitalInput hallEffect;

  private MAShuffleboard board;

  private SparkMaxPIDController pidController;

  public IntakePosition(){
    motor = new CANSparkMax(
      PortMap.Intake.intakePositionMotorID,
      MotorType.kBrushless);

    motor.setInverted(true);
  
    encoder = motor.getEncoder();

    hallEffect = new DigitalInput(
      PortMap.Intake.closingHallEffectPort);

    motor.setIdleMode(IdleMode.kCoast);

    encoder.setPosition(0);

    encoder.setPositionConversionFactor(IntakeConstants.POSITION_CONVERSION_FACTOR
    );

    board = new MAShuffleboard("IntakePosition");

    pidController = motor.getPIDController();
    pidController.setFeedbackDevice(encoder);

    pidController.setP(IntakeConstants.KP);
    pidController.setI(IntakeConstants.KI);
    pidController.setD(IntakeConstants.KD);

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
  
  public boolean isClose(){
    return !hallEffect.get()
    || getPosition() >= IntakeConstants.CLOSE_POSITION - 
      IntakeConstants.POSITION_TOLORANCE;
  }

  public boolean isMiddle(){
    return Math.abs(getPosition() - IntakeConstants.MIDDLE_POSITION) < 
      IntakeConstants.POSITION_TOLORANCE;
  }

  public boolean isOpen(){
    return Math.abs(getPosition() - IntakeConstants.OPEN_POSITION) < 
      IntakeConstants.POSITION_TOLORANCE;
  }

  public static IntakePosition getInstance() {
    if (openIntake == null) {
      openIntake = new IntakePosition();
    }
    return openIntake;
  }

  
  public boolean isAbleToClose() {
    return (
    ArmRotation.getInstance().getRotation() > ArmConstants.MIN_ROTATION_FOR_EXTENSTION_SAFTY_BUFFR
    && ArmRotation.getInstance().getSetPoint() > ArmConstants.MIN_ROTATION_FOR_EXTENSTION_SAFTY_BUFFR) 
    || (ArmRotation.getInstance().getRotation() < ArmConstants.MIN_ROTATION_FOR_CLOSING_INTAKE
    && ArmRotation.getInstance().getSetPoint() < ArmConstants.MIN_ROTATION_FOR_CLOSING_INTAKE);
  }

  @Override
  public void periodic() {
    if (!hallEffect.get()) {
      encoder.setPosition(IntakeConstants.CLOSE_POSITION);
    }

    board.addBoolean("isClose", isClose());
    board.addBoolean("isMiddle", isMiddle());
    board.addBoolean("isOpen", isOpen());

    board.addNum("position", getPosition());
    board.addNum("positionn degrees", Math.toDegrees(getPosition()));


    board.addBoolean("hall effect", !hallEffect.get());
  }
}