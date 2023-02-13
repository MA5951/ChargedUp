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

  private double motorAngleInRadians;
  private double currentPositionInRadians;

  public GripperCloseCommand() {
    gripperSubsystem = GripperSubsystem.getInstance();
    addRequirements(gripperSubsystem);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    motorAngleInRadians = gripperSubsystem.getCurrentEncoderPosition();
    currentPositionInRadians = gripperSubsystem.getCurrentEncoderPosition();
    if(Math.abs(currentPositionInRadians - GripperConstants.closePosition) 
      > GripperConstants.gripperTolerance) {
      if (motorAngleInRadians >= GripperConstants.coneAngle) {
        gripperSubsystem.setPower(GripperConstants.coneAngle);
      } else if (motorAngleInRadians >= GripperConstants.cudeAngle) {
        gripperSubsystem.setPower(GripperConstants.cubePower);
      } else {
        gripperSubsystem.setPower(GripperConstants.closingPower);
      }
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
