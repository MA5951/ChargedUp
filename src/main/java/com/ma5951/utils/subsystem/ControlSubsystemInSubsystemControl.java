package com.ma5951.utils.subsystem;

public interface ControlSubsystemInSubsystemControl extends MotorSubsystem{
  public void calculate(double setPoint); 

  /**
   * should be returing flase if you don't want the movment to stop
   * @return
   */
  public boolean atPoint();
}
