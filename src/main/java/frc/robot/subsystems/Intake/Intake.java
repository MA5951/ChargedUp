package frc.robot.subsystems.Intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PortMap;

public class Intake extends SubsystemBase{

  private static Intake intake;

  private TalonFX master;
  private TalonFX slave;


  private Intake() {
    master = new TalonFX(PortMap.Intake.intakeMotor1ID);
    slave = new TalonFX(PortMap.Intake.intakeMotor2ID);

    slave.follow(master);
  }

  public void setPower(double power){
    master.set(ControlMode.PercentOutput, power);
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
