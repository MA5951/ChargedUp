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

public class ArmRotation extends SubsystemBase implements ControlSubsystemInSubsystemControl{
  /** Creates a new ArmRotation. */
  private CANSparkMax motor;
  private DigitalInput hallEffect;

  private RelativeEncoder encoder;
  private SparkMaxPIDController pidController;

  private MAShuffleboard board;
  private String kp = "kp";
  private String ki = "ki";
  private String kd = "kd";

  private static ArmRotation armRotation;

  public ArmRotation() {
    motor = new CANSparkMax(ArmPortMap.rotationMotorID, MotorType.kBrushless);
    hallEffect = new DigitalInput(ArmPortMap.rotationHallEffectID);
    encoder = motor.getAlternateEncoder(ArmConstants.kCPR);

    pidController = motor.getPIDController();
    pidController.setFeedbackDevice(encoder);

    encoder.setPositionConversionFactor(2 * Math.PI);

    pidController.setP(ArmConstants.armRotationKp);
    pidController.setI(ArmConstants.armRotationKi);
    pidController.setD(ArmConstants.armRotationKd);

    board = new MAShuffleboard("ArmRotation");
    board.addNum(kp, ArmConstants.armRotationKp);
    board.addNum(ki, ArmConstants.armRotationKi);
    board.addNum(kd, ArmConstants.armRotationKd);
  }

  /**
   * @return radians
   */
  public double getRotation() {
    return encoder.getPosition();
  }

  @Override
  public void setVoltage(double voltage) {
    motor.set(voltage / 12.0);
  }

  @Override
  public boolean canMove() {
    return true;
  }

  @Override
  public void calculate(double setPoint) {
    pidController.setReference(setPoint, ControlType.kPosition,
    0, getFeed(), ArbFFUnits.kPercentOut);
  }

  @Override
  public boolean atPoint() {
    return false; //TODO
  }

  public double getCenterOfMass(double extenstion) {
    // TODO graph on excel
    return 0;
  }

  public double getFeed() {
    return (((ArmConstants.armMass * 9.8) * 
      getCenterOfMass(ArmExtenstion.getInstance().getExtenstion()))
      / ArmConstants.armRotationRadiusOfTheWheel +
      ArmConstants.armMass * 
      SwerveDrivetrainSubsystem.getInstance().getRadialAcceleration())
      * ArmConstants.armRotationNewtonToPercentage;
  }

  public static ArmRotation getInstance() {
    if (armRotation == null) {
      armRotation = new ArmRotation();
    }
    return armRotation;
  }

  @Override
  public void periodic() {
    pidController.setP(board.getNum(kp));
    pidController.setI(board.getNum(ki));
    pidController.setD(board.getNum(kd));
    board.addNum("rotation in dagrees", Math.toDegrees(getRotation()));
    if (hallEffect.get()) {
      encoder.setPosition(ArmConstants.armRotationStartPose);
    }
  }
}
