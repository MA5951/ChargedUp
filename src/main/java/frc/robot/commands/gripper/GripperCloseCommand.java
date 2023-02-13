// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.gripper;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.gripper.GripperConstants;
import frc.robot.subsystems.gripper.GripperSubsystem;

public class GripperCloseCommand extends CommandBase {
  /** Creates a new GripperCommand. */
  // Declare the subsystem
  private GripperSubsystem gripperSubsystem;

  private double lastCurrent;
  private boolean isCone;

  public GripperCloseCommand() {
    gripperSubsystem = GripperSubsystem.getInstance();
    addRequirements(gripperSubsystem);
  }

  @Override
  public void initialize() {
    lastCurrent = 0;
    isCone = false;
  }

  @Override
  public void execute() {
    double motorAngleInRadians = gripperSubsystem.getCurrentEncoderPosition();
    double PositionInRadians = gripperSubsystem.getCurrentEncoderPosition();
    double current = gripperSubsystem.getMotorCurrent();
    if (Math.abs(lastCurrent - current) >= GripperConstants.currentJump) {
      isCone = true;
    }
    if (isCone) {
      gripperSubsystem.setPower(GripperConstants.conePower);
    } else {
        if(Math.abs(PositionInRadians - GripperConstants.closePosition) 
          > GripperConstants.gripperTolerance) {
        if (motorAngleInRadians >= GripperConstants.cudeAngle) {
          gripperSubsystem.setPower(GripperConstants.cubePower);
        } else {
          gripperSubsystem.setPower(GripperConstants.closingPower);
        }
      }
    }
    lastCurrent = current;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return gripperSubsystem.getCurrentEncoderPosition()
      >= GripperConstants.cudeAngle || isCone;
  }
}
