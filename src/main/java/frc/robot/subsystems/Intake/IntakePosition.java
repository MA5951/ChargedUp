// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.ma5951.utils.MAShuffleboard;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakePosition extends SubsystemBase {

  private static IntakePosition openIntake;

  private CANSparkMax motor;
  private RelativeEncoder encoder;

  private DigitalInput hallEffect;

  private MAShuffleboard openIntakeShuffleboard;


  public IntakePosition(){
    motor = new CANSparkMax(IntakePortMap.OpenAndCloseIntakeMotorID, MotorType.kBrushless);
    encoder = motor.getEncoder();

    hallEffect = new DigitalInput(IntakePortMap.isCloseHallEffectChanelle);

    motor.setIdleMode(IdleMode.kCoast);
    encoder.setPositionConversionFactor(
      2 * Math.PI * (1 / IntakeConstants.ticksPerRound) * IntakeConstants.gear
    );
    resetEncoder();
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
    openIntakeShuffleboard.addBoolean("isClose", isClose());
    openIntakeShuffleboard.addBoolean("isOpen", isOpen());
    openIntakeShuffleboard.addBoolean("isMiddle", isMiddle());

    openIntakeShuffleboard.addNum("position", encoder.getPosition());
  }
}