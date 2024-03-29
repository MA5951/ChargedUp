// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.ma5951.utils.MAShuffleboard;
import com.ma5951.utils.RobotConstants;
import com.ma5951.utils.subsystem.ControlSubsystemInSubsystemControl;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PortMap;

public class ArmExtenstion extends SubsystemBase implements ControlSubsystemInSubsystemControl {
  /** Creates a new TelescopicArm. */
  private CANSparkMax motor;
  private RelativeEncoder encoder;
  private DigitalInput hallEffect;

  private SparkMaxPIDController pidController;
  
  private MAShuffleboard board;

  private double setPoint = 0;

  private static ArmExtenstion armExtenstion;

  public double defultPower = 0;

  private ArmExtenstion() {
    motor = new CANSparkMax(PortMap.Arm.extenstionMotorID, MotorType.kBrushless);
    encoder = motor.getEncoder();
    hallEffect = new DigitalInput(PortMap.Arm.extenstionHallEffectPort);

    encoder.setPositionConversionFactor(
      (ArmConstants.ARM_EXTENSTION_DIAMETER_OF_THE_WHEEL * Math.PI) / 10);
    pidController = motor.getPIDController();

    motor.setIdleMode(IdleMode.kBrake);
    pidController.setFeedbackDevice(encoder);

    pidController.setP(ArmConstants.ARM_EXUTENSTION_KP);
    pidController.setI(ArmConstants.ARM_EXTENSTION_KI);
    pidController.setD(ArmConstants.ARM_EXTENSTION_KD);

    board = new MAShuffleboard("ArmExtenstion");
  }

  /**
   * @return meters
   */
  public double getExtenstion() {
    return encoder.getPosition();
  }

  /**
   * @return m/s
   */
  public double getVelocity() {
    return encoder.getVelocity();
  }

  @Override
  public void setVoltage(double voltage) {
    motor.set(voltage / RobotConstants.MAX_VOLTAGE);
  }

  public void setSetpoint(double setPoint) {
    this.setPoint = setPoint;
  }

  public double getSetpoint() {
    return setPoint;
  }

  public boolean isAbleToChangeExtenstion() {
    return (ArmRotation.getInstance().getRotation() >
      ArmConstants.MIN_ROTATION_FOR_EXTENSTION
      || (getExtenstion() < ArmConstants.MIN_EXTENSTION_FOR_ROTATION
      && setPoint < ArmConstants.MIN_EXTENSTION_FOR_ROTATION)
      )
      && setPoint >= 0
      && setPoint <= ArmConstants.ARM_EXTENTION_MAX_POSE;
  }


  @Override
  public void calculate(double setPoint) {
    this.setPoint = setPoint;
    if (!isAbleToChangeExtenstion()) {
      setPower(defultPower);
    } else {
      pidController.setReference(setPoint, ControlType.kPosition);
    }
  }

  public boolean atPoint() {
    return Math.abs(encoder.getPosition() - setPoint)
    < ArmConstants.ARM_EXTENSTION_TOLERANCE;
  }

  public static ArmExtenstion getInstance() {
    if (armExtenstion == null) {
      armExtenstion = new ArmExtenstion();
    }
    return armExtenstion;
  }

  @Override
  public boolean canMove() {
    return true;
  }

  @Override
  public void periodic() {
    board.addNum("pose in extenstion", getExtenstion());

    if (!hallEffect.get()) {
      encoder.setPosition(0);
    }
  }
}
