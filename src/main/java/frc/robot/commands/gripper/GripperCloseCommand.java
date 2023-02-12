// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.gripper;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.gripper.GripperConstants;
import frc.robot.subsystems.gripper.GripperSubsystem;

public class GripperCloseCommand extends CommandBase {
  /** Creates a new GripperCommand. */
  // Declare the subsystem
  GripperSubsystem gripperSubsystem;
  // create "lastCurrent" variable
  double lastCurrent;
  // create "motorCurrent" variable
  double motorCurrent;
  // create "motorTicks" variable
  double motorTicks;

  boolean touched;

  double currentPosition;

  public GripperCloseCommand() {
    gripperSubsystem = GripperSubsystem.getInstance();
    addRequirements(gripperSubsystem);
  }

  @Override
  public void initialize() {
    touched = false;
  }

  @Override
  public void execute() {
    motorTicks = gripperSubsystem.getMotorTicks();
    
    motorCurrent = gripperSubsystem.getMotorCurrent();
    currentPosition = gripperSubsystem.getCurrentEncoderPosition();
    if(Math.abs(currentPosition - GripperConstants.closePosition)
      > GripperConstants.gripperTolerance){
      //checks if the gripper touched an object
      if (motorCurrent - lastCurrent > GripperConstants.currentJump) {
        touched = true;
      }
      if(touched){
        if (motorTicks >= 1500) {
          gripperSubsystem.setPower(GripperConstants.conePower);
        } else if (motorTicks >= 1000) {
          gripperSubsystem.setPower(GripperConstants.cubePower);
        }
      }
      else{
        gripperSubsystem.setPower(GripperConstants.closingPower);
      }
      lastCurrent = gripperSubsystem.getMotorCurrent();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    gripperSubsystem.setPower(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
