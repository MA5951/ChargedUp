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
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Intake.IntakeConstants;
import frc.robot.PortMap;
import frc.robot.subsystems.Intake.IntakePosition;
import frc.robot.subsystems.gripper.GripperConstants;
import frc.robot.subsystems.gripper.GripperSubsystem;

public class ArmRotation extends SubsystemBase implements ControlSubsystemInSubsystemControl{
  /** Creates a new ArmRotation. */
  private CANSparkMax motor;
  private DigitalInput hallEffect;

  private RelativeEncoder encoder;
  private SparkMaxPIDController pidController;

  private MAShuffleboard board;

  private double setPoint = ArmConstants.ARM_ROTATION_START_POSE;

  private static ArmRotation armRotation;

  private ArmRotation() {
    motor = new CANSparkMax(PortMap.Arm.rotationMotorID, MotorType.kBrushless);
    hallEffect = new DigitalInput(PortMap.Arm.rotationHallEffectPort);
    encoder = motor.getAlternateEncoder(ArmConstants.kCPR);

    motor.setIdleMode(IdleMode.kCoast);
    pidController = motor.getPIDController();

    encoder.setPositionConversionFactor(2 * Math.PI);
    encoder.setVelocityConversionFactor(2 * Math.PI / 60);

    pidController.setFeedbackDevice(encoder);
    encoder.setPosition(ArmConstants.ARM_ROTATION_START_POSE);

    pidController.setP(ArmConstants.ARM_ROTATION_KP);
    pidController.setI(ArmConstants.ARM_ROTATION_KI);
    pidController.setD(ArmConstants.ARM_ROTATION_KD);

    board = new MAShuffleboard("ArmRotation");
  }

  /**
   * @return radians
   */
  public double getRotation() {
    return encoder.getPosition();
  }

  public void setSetpoint(double setPoint) {
    this.setPoint = setPoint;
  }

  public double getSetPoint() {
    return setPoint;
  }

  @Override
  public void setVoltage(double voltage) {
    motor.set(voltage / RobotConstants.MAX_VOLTAGE);
  }

  public boolean isAbleToChangeRotation() {
    return (IntakePosition.getInstance().getPosition() 
            < IntakeConstants.MIDDLE_POSITION +
            IntakeConstants.POSITION_TOLORANCE || (
        getRotation() > ArmConstants.MIN_ROTATION_FOR_EXTENSTION_SAFTY_BUFFR
        && setPoint > ArmConstants.MIN_ROTATION_FOR_EXTENSTION_SAFTY_BUFFR
      ))
      && (ArmExtenstion.getInstance().getExtenstion() < 
      ArmConstants.MIN_EXTENSTION_FOR_ROTATION
      || getRotation() >= ArmConstants.MIN_ROTATION_FOR_EXTENSTION_SAFTY_BUFFR)
      && setPoint >= ArmConstants.ARM_ROTATION_START_POSE
      && setPoint <= ArmConstants.ARM_ROTATION_MAX_POSE
      && (
        (GripperSubsystem.getInstance().getCurrentEncoderPosition()
          < GripperConstants.INTAKE_POSITION) ||
        (getRotation() > ArmConstants.MIN_ROTATION_FOR_EXTENSTION_SAFTY_BUFFR
        && setPoint > ArmConstants.MIN_ROTATION_FOR_EXTENSTION_SAFTY_BUFFR)
      ); 
  }

  @Override
  public void calculate(double setPoint) {
    this.setPoint = setPoint;
    if (!isAbleToChangeRotation()) {
      setPower(getFeed());
    } else {
      pidController.setReference(setPoint, ControlType.kPosition,
        0, getFeed(), ArbFFUnits.kPercentOut);
    }
  }

  public boolean atPoint() {
    return Math.abs(encoder.getPosition() - setPoint)
      < ArmConstants.ARM_ROTATION_TOLERANCE;
  }

  public double getCenterOfMass() {
    double x = ArmExtenstion.getInstance().getExtenstion();
    return (0.475 * x + 0.248
      + 0.4226 * x + 0.1732) / 2.0;
  }

  public double getAngularVelocity() {
    return encoder.getVelocity();
  }

  public double getFeed() {
    double mass =
      (2 * ArmConstants.ARM_MASS + ArmConstants.CONE_MASS)
      / 2.0;
    double dis = getCenterOfMass();
    double GT = (mass * RobotConstants.KGRAVITY_ACCELERATION) * Math.cos(getRotation()) * dis;
    double angularMomentum = ArmConstants.ARM_MASS * getAngularVelocity();
    return (GT + dis * (-angularMomentum / 
      RobotConstants.KDELTA_TIME))
      * ArmConstants.ARM_ROTATION_KT;
  }

  public static ArmRotation getInstance() {
    if (armRotation == null) {
      armRotation = new ArmRotation();
    }
    return armRotation;
  }

  @Override
  public boolean canMove() {
    return true;
  }

  @Override
  public void periodic() {
    board.addNum("rotation in dagrees", Math.toDegrees(getRotation()));
    board.addNum("rotation in radians", getRotation());

    board.addBoolean("hallEffect", !hallEffect.get());

    board.addBoolean("at point", atPoint());
  }
}
