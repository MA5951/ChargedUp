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

  private double setPoint = ArmConstants.ARM_ROTATION_START_POSE;

  private static ArmRotation armRotation;

  private ArmRotation() {
    motor = new CANSparkMax(PortMap.Arm.rotationMotorID, MotorType.kBrushless);
    hallEffect = new DigitalInput(PortMap.Arm.rotationHallEffectPort);
    encoder = motor.getAlternateEncoder(ArmConstants.kCPR);

    motor.setIdleMode(IdleMode.kBrake);
    pidController = motor.getPIDController();
    pidController.setFeedbackDevice(encoder);

    encoder.setPositionConversionFactor(2 * Math.PI);
    encoder.setVelocityConversionFactor(2 * Math.PI / 60);

    pidController.setP(ArmConstants.ARM_ROTATION_KP);
    pidController.setI(ArmConstants.ARM_ROTATION_KI);
    pidController.setD(ArmConstants.ARM_ROTATION_KD);

    board = new MAShuffleboard("ArmRotation");
    board.addNum(kp, ArmConstants.ARM_ROTATION_KP);
    board.addNum(ki, ArmConstants.ARM_ROTATION_KI);
    board.addNum(kd, ArmConstants.ARM_ROTATION_KD);
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
            < IntakeConstants.MiddlePosition +
            IntakeConstants.positionTolorance || (
        getRotation() > ArmConstants.MIN_ROTATION_FOR_EXTENSTION_SAFTY_BUFFR
        && setPoint > ArmConstants.MIN_ROTATION_FOR_EXTENSTION_SAFTY_BUFFR
      ))
      && (ArmExtenstion.getInstance().getExtenstion() < 
      ArmConstants.MIN_EXTENSTION_FOR_ROTATION
      || getRotation() > ArmConstants.MIN_ROTATION_FOR_EXTENSTION_SAFTY_BUFFR)
      && setPoint < ArmConstants.ARM_ROTATION_START_POSE
      && setPoint > ArmConstants.ARM_ROTATION_MAX_POSE;
  }

  @Override
  public void calculate(double setPoint) {
    this.setPoint = setPoint;
    double useSetPoint = this.setPoint;
    if (!isAbleToChangeRotation()) {
      useSetPoint = getRotation();
    }
    pidController.setReference(useSetPoint, ControlType.kPosition,
      0, getFeed(), ArbFFUnits.kVoltage);
  }

  public boolean atPoint() {
    return Math.abs(encoder.getPosition() - setPoint)
      < ArmConstants.ARM_ROTATION_TOLERANCE;
  }

  public double getCenterOfMass() {
    // if (ArmConstants.isThereCone) {
    //   return 0.475 * ArmExtenstion.getInstance().getExtenstion() + 0.248;
    // }
    // double mu1 = 
    //   ((GripperConstants.openPosition
    //   - GripperSubsystem.getInstance().getCurrentEncoderPosition()) / 
    //   (GripperConstants.openPosition - GripperConstants.closePosition));
    // double mu2 = (1 - Math.cos(mu1 * Math.PI)) / 2;
    // double x = ArmExtenstion.getInstance().getExtenstion();
    // double close = 0.4311 * x + 0.2035;
    // double open = 0.4226 * x + 0.1732;
    // double y = close - open;
    // return open + y * mu2;
    double x = ArmExtenstion.getInstance().getExtenstion();
    return (0.475 * x + 0.248
      + 0.4226 * x + 0.1732) / 2.0;
  }

  public double getAngularVelocity() {
    return encoder.getVelocity();
  }

  public double getFeed() {
    double mass =
      ArmConstants.isThereCone ? ArmConstants.ARM_MASS + ArmConstants.CONE_MASS :
      ArmConstants.ARM_MASS;
    double dis = getCenterOfMass();
    // double r = ArmConstants.armDistanceFromTheCenter + 
    //   Math.cos(getRotation()) * dis;
    // double aR = 
    //   Math.pow(SwerveDrivetrainSubsystem.getInstance().getAngularVelocity(), 2) * r;
    // double FR = (aR * mass) * Math.sin(getRotation());
    // double TR = FR * dis;
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
    pidController.setP(board.getNum(kp));
    pidController.setI(board.getNum(ki));
    pidController.setD(board.getNum(kd));
    board.addNum("rotation in dagrees", Math.toDegrees(getRotation()));
    if (hallEffect.get()) {
      encoder.setPosition(ArmConstants.ARM_ROTATION_START_POSE);
    }
  }
}
