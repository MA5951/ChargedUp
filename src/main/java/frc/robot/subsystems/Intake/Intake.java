package frc.robot.subsystems.Intake;

import com.ma5951.utils.MAShuffleboard;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{

  private static Intake intake;

  private CANSparkMax upperMotor;
  private CANSparkMax lowerMotor;

  private MAShuffleboard intakeShuffleboard;

  public Intake() {
    upperMotor = new CANSparkMax(IntakePortMap.UpperMotorID,MotorType.kBrushless);
    lowerMotor = new CANSparkMax(IntakePortMap.LowerMotorID,MotorType.kBrushless);

    intakeShuffleboard = new MAShuffleboard("Intake");
  }


  public void setUpperMotorVelocity(double velocity){
    upperMotor.set(velocity);
  }

  public void setUpperMotorVoltage(double voltage){
    upperMotor.setVoltage(voltage);
  }

  public void setLowerMotorVelocity(double velocity){
    lowerMotor.set(velocity);
  }

  public void setLowerMotorVoltage(double voltage){
    lowerMotor.setVoltage(voltage);
  }

  public boolean isGamePiceEntered(){
    if(upperMotor.getOutputCurrent() > 9 && upperMotor.getOutputCurrent()< 9 ){//9 is the voltage when game pice entered, and 5 is tolorance
      return true;
    }

    return false;
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
