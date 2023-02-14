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
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.gripper.GripperConstants;
import frc.robot.subsystems.gripper.GripperSubsystem;
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

  private double setPoint = 0;

  private static ArmRotation armRotation;

  public ArmRotation() {
    motor = new CANSparkMax(ArmPortMap.rotationMotorID, MotorType.kBrushless);
    hallEffect = new DigitalInput(ArmPortMap.rotationHallEffectID);
    encoder = motor.getAlternateEncoder(ArmConstants.kCPR);

    motor.setIdleMode(IdleMode.kBrake);
    pidController = motor.getPIDController();
    pidController.setFeedbackDevice(encoder);

    encoder.setPositionConversionFactor(2 * Math.PI);
    encoder.setVelocityConversionFactor(2 * Math.PI / 60);

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
    this.setPoint = setPoint;
    pidController.setReference(setPoint, ControlType.kPosition,
    0, getFeed(), ArbFFUnits.kVoltage);
  }

  public boolean atPoint() {
    return Math.abs(encoder.getPosition() - setPoint)
      < ArmConstants.armRotationTolerance;
  }

  public double getCenterOfMass() {
    if (ArmConstants.isThereCone) {
      return 0.475 * ArmExtenstion.getInstance().getExtenstion() + 0.248;
    }
    double mu1 = 
      ((GripperConstants.openPosition
      - GripperSubsystem.getInstance().getCurrentEncoderPosition()) / 
      (GripperConstants.openPosition - GripperConstants.closePosition));
    double mu2 = (1 - Math.cos(mu1 * Math.PI)) / 2;
    double x = ArmExtenstion.getInstance().getExtenstion();
    double close = 0.4311 * x + 0.2035;
    double open = 0.4226 * x + 0.1732;
    double y = close - open;
    return open + y * mu2;
  }

  public double getAngularVelocity() {
    return encoder.getVelocity();
  }

  public double getFeed() {
    double mass =
      ArmConstants.isThereCone ? ArmConstants.armMass + ArmConstants.coneMass :
      ArmConstants.armMass;
    double dis = getCenterOfMass();
    double r = ArmConstants.armDistanceFromTheCenter + 
      Math.cos(getRotation()) * dis;
    double aR = 
      Math.pow(SwerveDrivetrainSubsystem.getInstance().getAngularVelocity(), 2) * r;
    double FR = (aR * mass) * Math.sin(getRotation());
    double TR = FR * dis;
    double GT = (mass * 9.8) * Math.cos(getRotation()) * dis;
    double angularMomentum = ArmConstants.armMass * getAngularVelocity();
    return (GT - TR + dis * (-angularMomentum / 0.02))
      * ArmConstants.armRotationkT;
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
