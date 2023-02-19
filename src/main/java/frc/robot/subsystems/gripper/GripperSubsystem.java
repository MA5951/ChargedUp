package frc.robot.subsystems.gripper;

import com.ma5951.utils.MAShuffleboard;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PortMap;

public class GripperSubsystem extends SubsystemBase {

  private static GripperSubsystem gripperSubsystem;
  private CANSparkMax gripperMotor;
  private RelativeEncoder encoder;

  private MAShuffleboard board;

  private GripperSubsystem() {
    gripperMotor = new CANSparkMax(
      PortMap.Gripper.gripperMotorID, MotorType.kBrushless);
    encoder = gripperMotor.getEncoder();
    encoder.setPositionConversionFactor((1 / GripperConstants.TICKS_PER_ROUND)
      * GripperConstants.GEAR * 2 * Math.PI);
    board = new MAShuffleboard("gripper");

    // gripperMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 0);
  }

  public void setPower(double power){
    gripperMotor.set(power);
  }
  
  public double getMotorCurrent() {
    return gripperMotor.getOutputCurrent();
  }

  /**
   * @return radians
   */
  public double getCurrentEncoderPosition(){
    return encoder.getPosition();
  }

  public static GripperSubsystem getInstance() {
    if (gripperSubsystem == null) {
      gripperSubsystem = new GripperSubsystem();
    }
    return gripperSubsystem;
  }

  @Override
  public void periodic() {
    board.addNum("position radians", getCurrentEncoderPosition());
    board.addNum("MotorCurrent", getMotorCurrent());
  }
}
