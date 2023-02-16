package frc.robot.subsystems.Intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{

  private static Intake intake;

  private TalonFX motor;


  private Intake() {
    motor = new TalonFX(IntakePortMap.IntakemotorID);
  }

  public void setPower(double power){
    motor.set(ControlMode.PercentOutput, power);;
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
