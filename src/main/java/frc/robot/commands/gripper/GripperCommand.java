// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.gripper;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.gripper.GripperSubsystem;

public class GripperCommand extends CommandBase {
  /** Creates a new GripperCommand. */
  // Declare the subsystem
  GripperSubsystem gripperSubsystem;
  // create "lastCurrent" variable
  double lastCurrent;
  // create "motorCurrent" variable
  double motorCurrent;
  // create "motorTicks" variable
  double motorTicks;

  public GripperCommand() {
    gripperSubsystem = GripperSubsystem.getInstance();
    addRequirements(gripperSubsystem);
  }

  @Override
  public void initialize() {
    gripperSubsystem.closeGripper(0.5);
  }

  @Override
  public void execute() {
    motorTicks = gripperSubsystem.getMotorTicks();
    motorCurrent = gripperSubsystem.getMotorCurrent();
    if (motorCurrent < lastCurrent + 0.1) {
      if (motorTicks >= 1500 && motorTicks >= 1000) {
        gripperSubsystem.closeGripper(0.3);
      } else if (motorTicks >= 1000 && motorTicks >= 750) {
        gripperSubsystem.closeGripper(0.2);
      }
    }
    else{
      gripperSubsystem.closeGripper(0.5);
    }
    lastCurrent = gripperSubsystem.getMotorCurrent();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
