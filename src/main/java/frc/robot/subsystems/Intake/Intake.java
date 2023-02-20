package frc.robot.subsystems.Intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PortMap;

public class Intake extends SubsystemBase{

  private static Intake intake;

  private CANSparkMax master;
  private CANSparkMax slave;


  private Intake() {
    master = new CANSparkMax(PortMap.Intake.intakeMotor1ID, MotorType.kBrushless);
    slave = new CANSparkMax(PortMap.Intake.intakeMotor2ID, MotorType.kBrushless);

    slave.follow(master);

    // slave.setCANTimeout(300);
    // master.setCANTimeout(300);

    // master.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 0);
    // slave.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 0);
  }

  public void setPower(double power){
    master.set(power);
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
