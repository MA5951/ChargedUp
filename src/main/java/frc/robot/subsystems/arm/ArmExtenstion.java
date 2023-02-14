// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.ma5951.utils.MAShuffleboard;
import com.ma5951.utils.subsystem.ControlSubsystemInSubsystemControl;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.SwerveDrivetrainSubsystem;

public class ArmExtenstion extends SubsystemBase implements ControlSubsystemInSubsystemControl {
  /** Creates a new TelescopicArm. */
  private CANSparkMax motor;
  private RelativeEncoder encoder;
  private DigitalInput hallEffect;

  private SparkMaxPIDController pidController;
  
  private MAShuffleboard board;
  private String kp = "kp";
  private String ki = "ki";
  private String kd = "kd";

  private double setPoint = 0;

  private static ArmExtenstion armExtenstion;

  private ArmExtenstion() {
    motor = new CANSparkMax(ArmPortMap.extenstionMotorID, MotorType.kBrushless);
    encoder = motor.getAlternateEncoder(ArmConstants.kCPR);
    hallEffect = new DigitalInput(ArmPortMap.extenstionHallEffectID);

    pidController = motor.getPIDController();
    pidController.setFeedbackDevice(encoder);
    
    encoder.setPositionConversionFactor(ArmConstants.armExtenstionDiameterOfTheWheel * Math.PI);

    pidController.setP(ArmConstants.armExtenstionKp);
    pidController.setI(ArmConstants.armExtenstionKi);
    pidController.setD(ArmConstants.armExtenstionKd);

    board = new MAShuffleboard("ArmExtenstion");
    board.addNum(kp, ArmConstants.armExtenstionKp);
    board.addNum(ki, ArmConstants.armExtenstionKi);
    board.addNum(kd, ArmConstants.armExtenstionKd);
  }

  /**
   * @return meters
   */
  public double getExtenstion() {
    return encoder.getPosition();
  }

  @Override
  public void setVoltage(double voltage) {
    motor.set(voltage / 12.0);
  }

  @Override
  public void calculate(double setPoint) {
    pidController.setReference(setPoint, ControlType.kPosition,0,
    getFeed(), ArbFFUnits.kPercentOut);
  }

  public double getFeed() {
    double mass =
      ArmConstants.isThereCone ? ArmConstants.armExtestionMass + ArmConstants.coneMass :
      ArmConstants.armExtestionMass;
    double dis = ArmRotation.getInstance().getCenterOfMass();
    double r = ArmConstants.armDistanceFromTheCenter + 
      Math.cos(ArmRotation.getInstance().getRotation()) * dis;
    double aR = 
      Math.pow(SwerveDrivetrainSubsystem.getInstance().getAngularVelocity(), 2) * r;
    double FR = (aR * mass) * Math.cos(ArmRotation.getInstance().getRotation());
    return (Math.sin(ArmRotation.getInstance().getRotation()) * 
      mass * 9.8 + FR) * ArmConstants.armExtenstionKn;
  }

  @Override
  public boolean canMove() {
    return true;
  }

  public boolean atPoint() {
    return Math.abs(encoder.getPosition() - setPoint)
    < ArmConstants.armExtenstionTolerance;
  }

  public static ArmExtenstion getInstance() {
    if (armExtenstion == null) {
      armExtenstion = new ArmExtenstion();
    }
    return armExtenstion;
  }

  @Override
  public void periodic() {
    pidController.setP(board.getNum(kp));
    pidController.setI(board.getNum(ki));
    pidController.setD(board.getNum(kd));
    board.addNum("pose in extenstion", getExtenstion());
    if (hallEffect.get()) {
      encoder.setPosition(0);
    }
  }
}
