package frc.robot.subsystems.Intake;

import com.ma5951.utils.MAShuffleboard;
import com.ma5951.utils.RobotConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  enum IntakePosition{
    Close,
    Open,
    Middle
  }

  private static Intake intake;

  private CANSparkMax upperMotor;
  private CANSparkMax lowerMotor;

  private CANSparkMax openAndCloseIntakeMotor;
  private RelativeEncoder openAndCloseIntakeEncoder;

  private DigitalInput isCloseHallEffect;

  private MAShuffleboard intakShuffleboard;

  public Intake() {
    upperMotor = new CANSparkMax(IntakeConstants.UpperMotorID,MotorType.kBrushless);
    lowerMotor = new CANSparkMax(IntakeConstants.LowerMotorID,MotorType.kBrushless);

    openAndCloseIntakeMotor = new CANSparkMax(IntakeConstants.OpenAndCloseIntakeMotorID, MotorType.kBrushless);
    openAndCloseIntakeEncoder = openAndCloseIntakeMotor.getAlternateEncoder(RobotConstants.KTICKS_PER_PULSE);

    isCloseHallEffect = new DigitalInput(IntakeConstants.isCloseHallEffectChanelle);

    intakShuffleboard = new MAShuffleboard("Intake");
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

  public void setOpenAndCloseIntakeMotorVelocity(double velocity){
    openAndCloseIntakeMotor.set(velocity);
  }

  public void setOpenAndCloseIntakeMotorVoltage(double voltage){
    openAndCloseIntakeMotor.setVoltage(voltage);
  }

  public boolean isIntakeClose(){
    if(isCloseHallEffect.get()){
      openAndCloseIntakeEncoder.setPosition(IntakeConstants.ClosePosition);
      return true;
    }
    return false;
  }

  public void resetEncoder(){
    openAndCloseIntakeEncoder.setPosition(0);
  }

  public void openIntake(IntakePosition enumPosition, double velocity){

    double position = 0;

    if(enumPosition == IntakePosition.Open){
      position = IntakeConstants.OpenPosition;
    }
    else if(enumPosition == IntakePosition.Close){
      position = IntakeConstants.OpenPosition;
    }
    else if(enumPosition == IntakePosition.Middle){
      position = IntakeConstants.MiddlePosition;
    }

    if(position != openAndCloseIntakeEncoder.getPosition()){
      openAndCloseIntakeMotor.set(velocity);
    }

    openAndCloseIntakeMotor.set(0);
  }

  public boolean isGamePiceEntered(){
    if(openAndCloseIntakeMotor.getBusVoltage() >9-5 && openAndCloseIntakeMotor.getBusVoltage()<9+5 ){//9 is the voltage when game pice entered, and 5 is tolorance
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
    intakShuffleboard.addBoolean("isIntakeClose", isIntakeClose());
    intakShuffleboard.addNum("position", openAndCloseIntakeEncoder.getPosition());
    intakShuffleboard.addBoolean("isGamePiceEntered", isGamePiceEntered());
  }
}
