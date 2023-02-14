package frc.robot.subsystems.Intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{

  private static Intake intake;

  private CANSparkMax upperMotor;
  private CANSparkMax lowerMotor;


  private Intake() {
    upperMotor = new CANSparkMax(IntakePortMap.UpperMotorID,MotorType.kBrushless);
    lowerMotor = new CANSparkMax(IntakePortMap.LowerMotorID,MotorType.kBrushless);
  }

  public void setUpperMotorPower(double power){
    upperMotor.set(power);
  }

  public void setLowerMotorPower(double power){
    lowerMotor.set(power);
  }

  public static Intake getInstance() {
    if (intake == null) {
      intake = new Intake();
    }
    return intake;
  }

  @Override
  public void periodic() {
    
  }
}
