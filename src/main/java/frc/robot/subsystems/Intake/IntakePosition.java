// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.ma5951.utils.MAShuffleboard;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakePosition extends SubsystemBase {

  enum intakePosition{
    Close,
    Open,
    Middle
  }

  private static IntakePosition openIntake;

  private CANSparkMax openAndCloseIntakeMotor;
  private RelativeEncoder openAndCloseIntakeEncoder;

  private DigitalInput isCloseHallEffect;

  private MAShuffleboard openIntakeShuffleboard;

  private ArmFeedforward feed;


  public IntakePosition(){
    openAndCloseIntakeMotor = new CANSparkMax(IntakePortMap.OpenAndCloseIntakeMotorID, MotorType.kBrushless);
    openAndCloseIntakeEncoder = openAndCloseIntakeMotor.getAlternateEncoder(IntakeConstants.kCPR);

    isCloseHallEffect = new DigitalInput(IntakePortMap.isCloseHallEffectChanelle);

    feed = new ArmFeedforward(0, IntakeConstants.kG, 0);

    openAndCloseIntakeMotor.setIdleMode(IdleMode.kCoast);
    openAndCloseIntakeEncoder.setPositionConversionFactor(2*Math.PI/IntakeConstants.ticksPerRound);

    resetEncoder();
  }


  public void setOpenAndCloseIntakeMotorVelocity(double velocity){
    openAndCloseIntakeMotor.set(velocity);
  }

  public void setOpenAndCloseIntakeMotorVoltage(double voltage){
    openAndCloseIntakeMotor.setVoltage(voltage);
  }

  public void resetEncoder(){
    openAndCloseIntakeEncoder.setPosition(0);
  }


  //10 is tolorance
  public void openIntake(double velocity){
    if(openAndCloseIntakeEncoder.getPosition()< IntakeConstants.ClosePosition+10 && openAndCloseIntakeEncoder.getPosition()> IntakeConstants.ClosePosition-10){
      openAndCloseIntakeMotor.set(0);
      openAndCloseIntakeEncoder.setPosition(IntakeConstants.ClosePosition);
    }

    openAndCloseIntakeMotor.set(-velocity);
  }

  public void middleIntake(double velocity){
    if(openAndCloseIntakeEncoder.getPosition()> IntakeConstants.MiddlePosition+10){
      openAndCloseIntakeMotor.set(-velocity);
    }
    else if(openAndCloseIntakeEncoder.getPosition()< IntakeConstants.MiddlePosition-10){
      openAndCloseIntakeMotor.set(velocity);
    }
    else if(openAndCloseIntakeEncoder.getPosition()< IntakeConstants.MiddlePosition+10 && openAndCloseIntakeEncoder.getPosition()> IntakeConstants.MiddlePosition-10){
      openAndCloseIntakeMotor.set(-feed.calculate((openAndCloseIntakeEncoder.getPosition())/IntakeConstants.ticksPerRound, 0));
    }

    openAndCloseIntakeMotor.set(0);

  }

  public void closeIntake(double velocity){
    if(openAndCloseIntakeEncoder.getPosition()< IntakeConstants.OpenPosition+10 && openAndCloseIntakeEncoder.getPosition()> IntakeConstants.OpenPosition-10){
      openAndCloseIntakeMotor.set(0);
    }

    openAndCloseIntakeMotor.set(velocity);
  }

  public void setPower(double power){
    openAndCloseIntakeMotor.set(power);
  }

  public static IntakePosition getInstance() {
    if (openIntake == null) {
      openIntake = new IntakePosition();
    }
    return openIntake;
  }



  @Override
  public void periodic() {
    if (isCloseHallEffect.get()) {
      openAndCloseIntakeEncoder.setPosition(IntakeConstants.ClosePosition);
    }
  }
}
