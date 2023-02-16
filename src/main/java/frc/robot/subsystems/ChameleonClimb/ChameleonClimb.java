// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ChameleonClimb;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PortMap;


public class ChameleonClimb extends SubsystemBase {
  /** Creates a new ChameleonClimb. */
  private static ChameleonClimb chameleonClimb;
  private CANSparkMax graspingMotor;

  public ChameleonClimb() {
    graspingMotor = new CANSparkMax(
      PortMap.ChameleonClimb.graspingMotorID, MotorType.kBrushless);
  }

  public void setMotor(double power){
    graspingMotor.set(power);
  }

  public double getCurrentStator(){
    return graspingMotor.getOutputCurrent();
  }

  public static ChameleonClimb getInstance() {
    if (chameleonClimb == null) {
      chameleonClimb = new ChameleonClimb();
    }
    return chameleonClimb;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
