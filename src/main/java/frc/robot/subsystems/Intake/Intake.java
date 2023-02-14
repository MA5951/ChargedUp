package frc.robot.subsystems.Intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{

  private static Intake intake;

  private CANSparkMax motor;


  private Intake() {
    motor = new CANSparkMax(IntakePortMap.IntakemotorID, MotorType.kBrushless);
  }

  public void setPower(double power){
    motor.set(power);
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
