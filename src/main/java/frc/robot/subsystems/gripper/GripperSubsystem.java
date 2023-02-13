package frc.robot.subsystems.gripper;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GripperSubsystem extends SubsystemBase {

  private static GripperSubsystem gripperSubsystem;
  private CANSparkMax gripperMotor;
  private RelativeEncoder encoder;

  public GripperSubsystem() {
    // Initialize the motor
    gripperMotor = new CANSparkMax(GripperProtMap.GripperMotorId,
      MotorType.kBrushless);
    encoder = gripperMotor.getEncoder();
    encoder.setPositionConversionFactor((1 / GripperConstants.kCPR)
    * GripperConstants.gear * 2 * Math.PI);
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
    // This method will be called once per scheduler run
  }
}
