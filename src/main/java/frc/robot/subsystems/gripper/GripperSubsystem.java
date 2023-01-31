package frc.robot.subsystems.gripper;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GripperSubsystem extends SubsystemBase {

  private static GripperSubsystem gripperSubsystem;
  private CANSparkMax gripperMotor;

  public GripperSubsystem() {
    // initialize the subsystem
    GripperSubsystem gripperSubsystem = new GripperSubsystem(); 
    // Initialize the motor
    gripperMotor = new CANSparkMax(1, MotorType.kBrushless);
  }

  public void closeGripper(double power) {
    gripperMotor.set(power);
  }

  public void openGripper(double power) {
    gripperMotor.set(-power);
  }
  
  public double getMotorTicks() {
    return gripperMotor.getEncoder().getPosition();
  }

  public double getMotorCurrent() {
    return gripperMotor.getOutputCurrent();
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
