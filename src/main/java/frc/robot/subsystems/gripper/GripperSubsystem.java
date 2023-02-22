package frc.robot.subsystems.gripper;

import com.ma5951.utils.MAShuffleboard;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PortMap;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmRotation;

public class GripperSubsystem extends SubsystemBase {

  private static GripperSubsystem gripperSubsystem;
  private CANSparkMax gripperMotor;
  private RelativeEncoder encoder;

  private MAShuffleboard board;

  private SparkMaxPIDController pid;

  private double setPoint;

  private GripperSubsystem() {
    gripperMotor = new CANSparkMax(
      PortMap.Gripper.gripperMotorID, MotorType.kBrushless);
    encoder = gripperMotor.getEncoder();
    gripperMotor.setInverted(true);
    encoder.setPositionConversionFactor((GripperConstants.POSITION_CONVERSION_FACTOR));
    board = new MAShuffleboard("gripper");
    encoder.setPosition(0);
    
    gripperMotor.setSmartCurrentLimit(30);
    pid = gripperMotor.getPIDController();

    pid.setP(GripperConstants.kP);
    pid.setI(GripperConstants.kI);
    pid.setD(GripperConstants.kD);

    encoder.setPosition(0);
  }

  public void setPower(double power){
    gripperMotor.set(power);
  }
  
  public double getMotorCurrent() {
    return gripperMotor.getOutputCurrent();
  }

  public double getCurrentEncoderPosition(){
    return encoder.getPosition();
  }

  public void calculate(double setPoint){
    this.setPoint = setPoint;
    double useSetPoint = setPoint;
    if (!(ArmRotation.getInstance().getRotation() < 
      GripperConstants.MAX_ARM_ROTATION_FOR_GRIPPER
    || ArmRotation.getInstance().getRotation() > 
      ArmConstants.MIN_ROTATION_FOR_EXTENSTION)
    ) {
      useSetPoint = getCurrentEncoderPosition();
    }
    pid.setReference(useSetPoint, ControlType.kPosition);
  }

  public boolean atSetPoint(){
    return Math.abs(encoder.getPosition() - setPoint)
      < GripperConstants.GRIPPER_TOLERANCE;
  }

  public static GripperSubsystem getInstance() {
    if (gripperSubsystem == null) {
      gripperSubsystem = new GripperSubsystem();
    }
    return gripperSubsystem;
  }

  @Override
  public void periodic() {
    board.addNum("position", getCurrentEncoderPosition());
    board.addNum("MotorCurrent", getMotorCurrent());
 }
}
